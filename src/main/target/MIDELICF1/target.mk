F1_TARGETS  += $(TARGET)
FLASH_SIZE  = 128

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
			drivers/barometer_ms5611.c \
            drivers/compass_hmc5883l.c \
		    drivers/c2500.c \
			rx/frskyD_rx.c \
			telemetry/telemetry.c \
			telemetry/frsky.c \
            sensors/barometer.c \

