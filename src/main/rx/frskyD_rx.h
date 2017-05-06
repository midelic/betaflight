
/* Frsky-RX D16  
by Midelic on RCGroups
         -2016-
 version for cleanflight/betaflight
*/
#pragma once


#include <stdbool.h>
#include <stdint.h>

struct rxConfig_s;
struct rxRuntimeConfig_s;
void frskyD_Rx_Init(const struct rxConfig_s *rxConfig, struct rxRuntimeConfig_s *rxRuntimeConfig);
void frskyD_Rx_SetRCdata(uint16_t *rcData, const uint8_t *payload);
rx_spi_received_e frskyD_Rx_DataReceived(uint8_t *payload);
