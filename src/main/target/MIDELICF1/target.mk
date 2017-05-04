F1_TARGETS  += $(TARGET)
FLASH_SIZE  = 128

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
            drivers/compass_hmc5883l.c \
			drivers/cc2500.c\
            blackbox/blackbox.c \
            blackbox/blackbox_io.c \
            telemetry/telemetry.c \
			rx/frskyX_rx.c\
			rx/frskyD_rx.c\

