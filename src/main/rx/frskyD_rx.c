
/*
* FrskyX RX D8
*  By  Midelic
*  on RCGroups
*   -2017-
Version for cleanflight/betaflight FC
***************
*/


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"


#ifdef USE_RX_FRSKYD	
#include "build/build_config.h"
//#include "build/debug.h"

#include "drivers/gpio.h"
#include "drivers/io.h"

#include "common/utils.h"
#include "drivers/cc2500.h"
#include "drivers/system.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/frskyD_rx.h"
//
#include "sensors/battery.h"
#include "fc/config.h"
#include "config/feature.h"

//#include "telemetry/frsky.h"
//**********************
//#define AUTOBIND
#define RC_CHANNEL_COUNT 8
#define MAX_MISSING_PKT	100
//
#define SYNC 9000
#define FS_THR 960
//***************************
#define FLED_on      {IOHi(IOGetByTag(IO_TAG(FRSKY_LED_PIN)));}
#define FLED_off     {IOLo(IOGetByTag(IO_TAG(FRSKY_LED_PIN)));}
#define GDO_1        IORead(IOGetByTag(IO_TAG(GDO_0_PIN)))
//
#if defined PA_LNA
#define  TX_EN_on {IOHi(IOGetByTag(IO_TAG(TX_EN_PIN)));}
#define  TX_EN_off {IOLo(IOGetByTag(IO_TAG(TX_EN_PIN)));}
//
#define  RX_EN_on {IOHi(IOGetByTag(IO_TAG(RX_EN_PIN)));}
#define  RX_EN_off {IOLo(IOGetByTag(IO_TAG(RX_EN_PIN)));}
//
#if defined DIVERSITY 
#define ANT_SEL_on	{IOHi(IOGetByTag(IO_TAG(ANT_SEL_PIN)));}
#define ANT_SEL_off {IOLo(IOGetByTag(IO_TAG(ANT_SEL_PIN)));}
#endif	
#endif


#if defined STM32F10X_MD
#define FLASH_PAGE_SIZE   ((uint16_t)0x400)//1K-1024
#define FLASH_TO_RESERVE_FOR_CONFIG  ((uint16_t)0x800)
#define FRSKYD_FLASH ((uint16_t)0x400)
#else
#define FLASH_PAGE_SIZE   ((uint16_t)0x800)//2K-2048
#define FLASH_TO_RESERVE_FOR_CONFIG  ((uint16_t)0x1000)
#define FRSKYD_FLASH ((uint16_t)0x400)
#endif
#define FLASH_PAGE_COUNT 128
#define CONFIG_START_FLASH_ADDRESS (0x08000000 + (uint32_t)((FLASH_PAGE_SIZE * FLASH_PAGE_COUNT) - FLASH_TO_RESERVE_FOR_CONFIG))

static uint8_t ccLen;
static uint8_t channr;
static uint32_t missingPackets;
static uint8_t calData[255][3];
static uint8_t listLength;
static uint8_t cnt;
static uint16_t Servo_data[RC_CHANNEL_COUNT];
static uint8_t jumper;
static uint16_t c[8];
static uint32_t t_out;
static uint8_t Lqi;
static uint32_t packet_timer;
static uint8_t protocol_state;
static int16_t word_temp;
static uint32_t start_time;
//
static IO_t GdoPin ;
static IO_t BindPin = DEFIO_IO(NONE);
static IO_t FrskyLedPin;
#if defined PA_LNA
static IO_t TxEnPin;
static IO_t RxEnPin;
static IO_t AntSelPin;
static uint8_t lpass=1;
static uint8_t pass;
#endif	
static uint8_t *storage_ptr ;
bool bind_status;

#ifdef FRSKY_TELEMETRY
static uint8_t frame[20];
static int16_t RSSI_dBm;
static uint8_t telemetry_id;
static uint8_t telemetryRX;
static uint8_t v1;//A1
static uint8_t v2;//A2
static uint32_t time_t;

#if defined HUB
#define MAX_SERIAL_BYTES	64
uint8_t hub_index;
uint8_t idxx=0;
uint8_t idx_ok = 0;
uint8_t telemetry_expected_id = 0;
uint8_t srx_data[MAX_SERIAL_BYTES];//buffer for telemetry serial data
#endif
#endif
typedef struct {
	// persistent storage for frsky
	uint8_t hopData[50];
	uint8_t txid[2];
	int8_t  offset;
	#if defined FRSKY_FAILSAFE
	uint16_t fsData[RC_CHANNEL_COUNT];//failsafe data not used yet
	#endif
} STORAGE_FRSKY_DATA;

STORAGE_FRSKY_DATA storage_frsky;
rx_spi_received_e ret;

enum {
	STATE_INIT=0,
	STATE_BIND,
	STATE_UPDATE,
	STATE_DATA,
	STATE_TELEM
};

#if defined FRSKY_TELEMETRY	
static void compute_RSSIdbm(uint8_t *packet){ 
	if(packet[ccLen-2] >=128){
		RSSI_dBm =((((uint16_t)packet[ccLen-2])*18)>>5)- 82;
	}
	else{
		RSSI_dBm = ((((uint16_t)packet[ccLen-2])*18)>>5)+65;
	}
}
#if defined HUB	
static uint8_t frsky_append_hub_data(uint8_t *buf){
	if(telemetry_id == telemetry_expected_id)
	idx_ok=idxx;
	else// rx re-requests last packet
	idxx=idx_ok;

	telemetry_expected_id = (telemetry_id + 1) & 0x1F;		
	uint8_t index = 0;	
	for (uint8_t i=0;i<10;i++){
		if(idxx==hub_index){
			break;
		}
		buf[i]=srx_data[idxx];
		idxx = (idxx+1) & (MAX_SERIAL_BYTES-1);			
		index++;
	}
	return index;
}

#endif

static void telemetry_build_frame(uint8_t *packet){
#ifdef USE_ADC
	if(feature(FEATURE_VBAT)){	
		v1 =vbat;
		v2=0;
	}
	else
	v1=0;
#endif
	uint8_t bytes_used = 0;
	telemetry_id = packet[4];
	frame[0] = 0x11;//length
	frame[1] = storage_frsky.txid[0];
	frame[2] = storage_frsky.txid[1];	
	frame[3] = v1;//A1 voltages
	frame[4] = v2;//A2	
	frame[5] = (uint8_t)RSSI_dBm;
	#if defined HUB					
	bytes_used = frsky_append_hub_data(&frame[8]);			
	#endif
	frame[6] = bytes_used;
	frame[7] = telemetry_id;
	//
}


#endif

#if defined PA_LNA
static void RX_enable(){
	TX_EN_off;
	RX_EN_on;
}

static void TX_enable(){
	RX_EN_off;
	TX_EN_on;
}
#endif

static void initialize() {
	CC2500_Reset();
	cc2500_writeReg(CC2500_02_IOCFG0,   0x01);	
	cc2500_writeReg(CC2500_17_MCSM1,    0x0C);	
	cc2500_writeReg(CC2500_18_MCSM0,    0x18);	
	cc2500_writeReg(CC2500_06_PKTLEN,   0x19);	
	cc2500_writeReg(CC2500_08_PKTCTRL0, 0x05);	
	cc2500_writeReg(CC2500_3E_PATABLE,  0xFF);  
	cc2500_writeReg(CC2500_0B_FSCTRL1,  0x08);	
	cc2500_writeReg(CC2500_0C_FSCTRL0,  0x00);
	cc2500_writeReg(CC2500_0D_FREQ2,    0x5C);	
	cc2500_writeReg(CC2500_0E_FREQ1,    0x76);	
	cc2500_writeReg(CC2500_0F_FREQ0,    0x27);
	cc2500_writeReg(CC2500_10_MDMCFG4,  0xAA);	
	cc2500_writeReg(CC2500_11_MDMCFG3,  0x39);	
	cc2500_writeReg(CC2500_12_MDMCFG2,  0x11);	
	cc2500_writeReg(CC2500_13_MDMCFG1,  0x23);	
	cc2500_writeReg(CC2500_14_MDMCFG0,  0x7A);	
	cc2500_writeReg(CC2500_15_DEVIATN,  0x42);	
	cc2500_writeReg(CC2500_19_FOCCFG,   0x16);
	cc2500_writeReg(CC2500_1A_BSCFG,    0x6C);	
	cc2500_writeReg(CC2500_1B_AGCCTRL2, 0x03);	
	cc2500_writeReg(CC2500_1C_AGCCTRL1, 0x40);	
	cc2500_writeReg(CC2500_1D_AGCCTRL0, 0x91);	
	cc2500_writeReg(CC2500_21_FREND1,   0x56); 
	cc2500_writeReg(CC2500_22_FREND0,   0x10);	
	cc2500_writeReg(CC2500_23_FSCAL3,   0xA9);	
	cc2500_writeReg(CC2500_24_FSCAL2,   0x0A);	
	cc2500_writeReg(CC2500_25_FSCAL1,   0x00);	
	cc2500_writeReg(CC2500_26_FSCAL0,   0x11);	
	cc2500_writeReg(CC2500_29_FSTEST,   0x59);	
	cc2500_writeReg(CC2500_2C_TEST2,    0x88);	
	cc2500_writeReg(CC2500_2D_TEST1,    0x31);	
	cc2500_writeReg(CC2500_2E_TEST0,    0x0B);	
	cc2500_writeReg(CC2500_03_FIFOTHR,  0x07);	
	cc2500_writeReg(CC2500_09_ADDR, 0x00);	
	cc2500_strobe(CC2500_SIDLE);
	
	cc2500_writeReg(CC2500_07_PKTCTRL1,0x04);
	cc2500_writeReg(CC2500_0C_FSCTRL0, 0);
	for(uint8_t c=0;c<0xFF;c++){
		cc2500_strobe(CC2500_SIDLE);  
		cc2500_writeReg(CC2500_0A_CHANNR, c);
		cc2500_strobe(CC2500_SCAL);
		delayMicroseconds(900);
		calData[c][0] = cc2500_readReg(CC2500_23_FSCAL3);
		calData[c][1] = cc2500_readReg(CC2500_24_FSCAL2);	
		calData[c][2] = cc2500_readReg(CC2500_25_FSCAL1);
	}				
}

static void initialize_data(uint8_t adr){
	cc2500_writeReg(CC2500_0C_FSCTRL0,(uint8_t)storage_frsky.offset);
	cc2500_writeReg(CC2500_18_MCSM0,    0x8);	
	cc2500_writeReg(CC2500_09_ADDR, adr ? 0x03 : storage_frsky.txid[0]);	
	cc2500_writeReg(CC2500_07_PKTCTRL1,0x0D);
	cc2500_writeReg(CC2500_19_FOCCFG,   0x16);
	delay(10);
}




static void tunning(uint8_t *packet){

	cc2500_writeReg(CC2500_19_FOCCFG,   0x14);
	uint32_t time_tune=millis();
	storage_frsky.offset=-126;
	cc2500_writeReg(CC2500_0C_FSCTRL0,(uint8_t)storage_frsky.offset);
	cc2500_writeReg(CC2500_07_PKTCTRL1,0x0C);	
	cc2500_writeReg(CC2500_18_MCSM0,    0x8);	

	cc2500_strobe(CC2500_SIDLE);
	cc2500_writeReg(CC2500_23_FSCAL3,   calData[0][0]);	
	cc2500_writeReg(CC2500_24_FSCAL2,   calData[0][1]);
	cc2500_writeReg(CC2500_25_FSCAL1,   calData[0][2]);	
	cc2500_writeReg(CC2500_0A_CHANNR, 0);
	cc2500_strobe(CC2500_SFRX);
	cc2500_strobe(CC2500_SRX);

	while(1){
		if (storage_frsky.offset>= 126){
			storage_frsky.offset = -126;
		}
		if ((millis()-time_tune)>50) {
			time_tune=millis();
			storage_frsky.offset+=5;
			cc2500_writeReg(CC2500_0C_FSCTRL0,(uint8_t)storage_frsky.offset);
		}
		if(GDO_1){
			ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if (ccLen) {
				cc2500_readFifo(packet, ccLen);
				if(packet[ccLen-1] & 0x80) {
					if(packet[2]==0x01) {
						Lqi=packet[ccLen-1]& 0x7F;
						if(Lqi<50)
						break;
					}
				}
			}
		}
	}

}


static void getBind(uint8_t *packet)
{
	cc2500_strobe(CC2500_SIDLE);
	cc2500_writeReg(CC2500_23_FSCAL3,   calData[0][0]);	
	cc2500_writeReg(CC2500_24_FSCAL2,   calData[0][1]);	
	cc2500_writeReg(CC2500_25_FSCAL1,   calData[0][2]);	
	cc2500_writeReg(CC2500_0A_CHANNR, 0);
	cc2500_strobe(CC2500_SFRX);
	delayMicroseconds(20);//waiting flush FIFO
	//
	cc2500_strobe(CC2500_SRX);
	listLength = 0;
	bool eol = false;
	//len|bind |tx id|03|01|idx|h0|h1|h2|h3|h4|00|00|00|00|00|00|00|00|00|00|00|00|00|00|00|CHK1|CHK2|RSSI|LQI/CRC|
	// Start by getting bind packet 0 and the txid
	while (1) {
		if(GDO_1){
			ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if (ccLen) {
				cc2500_readFifo(packet, ccLen);
				if(packet[ccLen-1] & 0x80) {
					if(packet[2]==0x01) {
						if(packet[5]==0x00) {
							storage_frsky.txid[0] = packet[3];
							storage_frsky.txid[1]= packet[4];
							for (uint8_t n=0;n<5;n++) {
								storage_frsky.hopData[packet[5]+n] = packet[6+n];
							}
							break;
						}
					}
				}
			}
		}
	}

	for (uint8_t bindIdx=0x05; bindIdx<=120;bindIdx+=5) {
		while (1) {
			if(GDO_1){
				ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
				if (ccLen) {
					cc2500_readFifo(packet, ccLen);
					if(packet[ccLen-1] & 0x80) {
						if(packet[2]==0x01) {
							if((packet[3]==storage_frsky.txid[0]) && (packet[4]==storage_frsky.txid[1])) {
								if(packet[5]==bindIdx) {
									//
									#if defined DJTS								
									if(packet[5]==0x2D){
										for(uint8_t i=0;i<2;i++){
											storage_frsky.hopData[packet[5]+i]=packet[6+i];
										}
										listLength=47;
										eol=true;
										break;
									}
									#endif
									//
									for (uint8_t n=0;n<5;n++) {
										if (packet[6+n] == packet[ccLen-3]||(packet[6+n]==0)) {
											if(bindIdx>=0x2D){
												eol = true;
												listLength = packet[5]+n;
												break;
											}
										}
										storage_frsky.hopData[packet[5]+n] = packet[6+n];
									}
									break;
								}
							}
						}
					}
				}
			}
		}

		if (eol) break;
	}
	cc2500_strobe(CC2500_SIDLE);
}


static void nextChannel(uint8_t skip)
{
	channr += skip;
	if (channr>=listLength) channr -= listLength;
	cc2500_strobe(CC2500_SIDLE);
	cc2500_writeReg(CC2500_23_FSCAL3,   calData[storage_frsky.hopData[channr]][0]);
	cc2500_writeReg(CC2500_24_FSCAL2,   calData[storage_frsky.hopData[channr]][1]);
	cc2500_writeReg(CC2500_25_FSCAL1,   calData[storage_frsky.hopData[channr]][2]);
	cc2500_writeReg(CC2500_0A_CHANNR, storage_frsky.hopData[channr]);
	cc2500_strobe(CC2500_SFRX);
}

static uint8_t bind_jumper(void){
	if(!IORead(BindPin))
	{
		delayMicroseconds(10);
		return 1;
	}
	return  0;
}


void Store_EEPROM_data(uint32_t startAddress){	
	FLASH_Status status =0;
	storage_ptr=(uint8_t*)&storage_frsky;
	FLASH_Unlock();//unlock flash writing
	#if defined STM32F10X_MD
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	#else
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
	#endif
	while(status != FLASH_COMPLETE) {
		status = FLASH_ErasePage(startAddress);
	}
	startAddress+=2;
	status=0;	
	while(status!=FLASH_COMPLETE){
		for(uint8_t i=0;i<sizeof(storage_frsky);i++){;
			status = FLASH_ProgramHalfWord((startAddress+(2*i)),*(uint16_t *) (storage_ptr+i));
			if(status!=FLASH_COMPLETE)
			break;
		}
	}
	FLASH_Lock();//lock the flash for writing
}

void Read_EEPROM_data(uint32_t startAddress){
	startAddress+=2;				
	for(uint8_t i=0;i< sizeof(storage_frsky.hopData);i++){
		storage_frsky.hopData[i]= (*(uint8_t *)(startAddress+(2*i)));
	}
	startAddress+=(2*sizeof(storage_frsky.hopData));
	storage_frsky.txid[0]= (*(uint8_t *)(startAddress));
	storage_frsky.txid[1]= (*(uint8_t *)(startAddress+2));
	storage_frsky.offset = (*(uint8_t *)(startAddress+4));
	//for test
	//txid[0]=0xC0;
	//txid[1]=0x49;
	//offset=0;
	listLength=47;	
}


static void binding(uint8_t *packet)

{
	#ifndef AUTOBIND
	uint32_t address=CONFIG_START_FLASH_ADDRESS-FRSKYD_FLASH;
	#else
	jumper=1;
	#endif
	if(bind_status||jumper){
		FLED_on;
		tunning(packet);
		initialize_data(1);
		getBind(packet);
		#ifndef AUTOBIND
		Store_EEPROM_data(address);
		bind_status=false;
		jumper=0;
		#else
		uint8_t ctr=40;
		while(ctr--){
			FLED_on;
			delay(50);
			FLED_off;
			delay(50);
		}
		#endif
	}
	else
	//eeprom read			
	Read_EEPROM_data(address);

}


void frskyD_Rx_SetRCdata(uint16_t *rcData, const uint8_t *packet)
{
	c[0] = (uint16_t)(((packet[10] & 0x0F)<<8 | packet[6]));
	c[1] = (uint16_t)(((packet[10] & 0xF0)<<4 | packet[7]));
	c[2] = (uint16_t)(((packet[11] & 0x0F)<<8 | packet[8]));
	c[3] = (uint16_t)(((packet[11] & 0xF0)<<4 | packet[9]));
	c[4] = (uint16_t)(((packet[16] & 0x0F)<<8 | packet[12]));
	c[5] = (uint16_t)(((packet[16] & 0xF0)<<4 | packet[13]));
	c[6] = (uint16_t)(((packet[17] & 0x0F)<<8 | packet[14]));
	c[7] = (uint16_t)(((packet[17] & 0xF0)<<4 | packet[15]));

	for(uint8_t i=0;i<8;i++){
		word_temp=0.667*c[i];
		if ((word_temp>800) && (word_temp<2200))
		Servo_data[i]=word_temp;			
		rcData[i]=Servo_data[i];
	}
}

rx_spi_received_e frskyD_Rx_DataReceived(uint8_t *packet)
{
	ret = RX_SPI_RECEIVED_NONE;	

	switch(protocol_state)
	{
	case STATE_INIT:
		if((millis()-start_time)>10){
			initialize();
			protocol_state = STATE_BIND;
		}	
		break;
	case STATE_BIND:	
		binding(packet);
		initialize_data(0);
		protocol_state = STATE_UPDATE;
		nextChannel(1);//
		cc2500_strobe(CC2500_SRX);	
		ret = RX_SPI_RECEIVED_BIND;
		break;
	case STATE_UPDATE:
		packet_timer=micros();
		protocol_state = STATE_DATA;
		if(bind_status){
	    packet_timer=0;
	    t_out=50;
	    missingPackets=0;
		protocol_state = STATE_INIT;
		break;
		}
		//here FS code could be
	case STATE_DATA:
		if(GDO_1)
		{			
			ccLen = cc2500_readReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if (ccLen>20) 
			ccLen = 20;
			if (ccLen==20) 
			{		
				cc2500_readFifo(packet, ccLen);
				if(packet[ccLen-1] & 0x80) 
				{ 
					missingPackets = 0;	
					t_out=1;
					if (packet[0]==0x11) { 			
						if((packet[1]==storage_frsky.txid[0])&& (packet[2] == storage_frsky.txid[1]))
						{ 
							FLED_on;
							nextChannel(1);
							#ifdef FRSKY_TELEMETRY
							if((packet[3]%4)==2){
								telemetryRX=1;
								time_t=micros();
								compute_RSSIdbm(packet);
								telemetry_build_frame(packet);
								protocol_state=STATE_TELEM;
							}
							else
							#endif
							{
								cc2500_strobe(CC2500_SRX);
								protocol_state=STATE_UPDATE;
							}
							ret = RX_SPI_RECEIVED_DATA;
							packet_timer=micros();
						}
					}
				}
				
			}
			
		}
		if ((micros()-packet_timer)>(t_out*SYNC))
		{	
			#ifdef PA_LNA
			RX_enable();
			#endif
			if(t_out==1)
			{			
				#ifdef DIVERSITY//SE4311 chip						
				if(missingPackets>=2){				
					if(pass & 0x01){
						ANT_SEL_on;
					}
					else{					
						ANT_SEL_off;
					}
					pass++;
				}			
				#endif
				
				if(missingPackets>MAX_MISSING_PKT)
				t_out=50;
				
				missingPackets++;
				nextChannel(1);
				
			}
			else
			{
				if(cnt++ & 0x01){
				FLED_off;
				}
				else
				FLED_on;
				
				nextChannel(13);
			}
			
			cc2500_strobe(CC2500_SRX);
			protocol_state=STATE_UPDATE;
		}
		break;
#ifdef FRSKY_TELEMETRY
	case STATE_TELEM:
		if(telemetryRX){
			if((micros()-time_t)>=1380)
			{
				cc2500_strobe(CC2500_SIDLE);
				CC2500_SetPower(6);
				cc2500_strobe(CC2500_SFRX);
				#if defined PA_LNA
				TX_enable();
				#endif
				cc2500_strobe(CC2500_SIDLE);
				cc2500_writeFifo(frame, frame[0]+1);
				protocol_state=STATE_DATA;
				ret = RX_SPI_RECEIVED_DATA;	
				packet_timer=micros();
				telemetryRX=0;			
			}		
		}

		break;

#endif

	}	
	return ret;
}

void frskyD_Rx_Setup(rx_spi_protocol_e protocol)
{
	UNUSED(protocol);
	//gpio init here
	GdoPin = IOGetByTag(IO_TAG(GDO_0_PIN));
	BindPin = IOGetByTag(IO_TAG(BIND_PIN));
	FrskyLedPin=IOGetByTag(IO_TAG(FRSKY_LED_PIN));
	#if defined PA_LNA
	RxEnPin	= IOGetByTag(IO_TAG(RX_EN_PIN));
	TxEnPin = IOGetByTag(IO_TAG(TX_EN_PIN));		
	IOInit(RxEnPin,OWNER_RX_BIND,0);
	IOInit(TxEnPin,OWNER_RX_BIND,0);
	IOConfigGPIO(RxEnPin, IOCFG_OUT_PP);
	IOConfigGPIO(TxEnPin, IOCFG_OUT_PP);
	#endif
	#if defined DIVERSITY
	AntSelPin=IOGetByTag(IO_TAG(ANT_SEL_PIN));		
	IOInit(AntSelPin,OWNER_RX_BIND,0);
	IOConfigGPIO(AntSelPin, IOCFG_OUT_PP);
	#endif
	//
	IOInit(GdoPin, OWNER_RX_BIND, 0);
	IOInit(BindPin, OWNER_RX_BIND, 0);
	IOInit(FrskyLedPin, OWNER_LED,0);	
	//
	IOConfigGPIO(GdoPin, IOCFG_IN_FLOATING);
	IOConfigGPIO(BindPin, IOCFG_IPU);
	IOConfigGPIO(FrskyLedPin, IOCFG_OUT_PP);
	start_time=millis();
	packet_timer=0;
	t_out=50;
	jumper = 0;
	missingPackets=0;
	jumper=bind_jumper();
	protocol_state=STATE_INIT;
	#if defined DIVERSITY
	ANT_SEL_on;
	#endif
	#if defined PA_LNA
	RX_enable();	
	#endif
	//if(!frskySpiDetect())//detect spi working routine
	//return;
}

void frskyD_Rx_Init(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
	rxRuntimeConfig->channelCount = RC_CHANNEL_COUNT;
	frskyD_Rx_Setup((rx_spi_protocol_e)rxConfig->rx_spi_protocol);
}
#endif

