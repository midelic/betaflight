F3_TARGETS  += $(TARGET)
FEATURES  = VCP ONBOARDFLASH

TARGET_SRC = \
            drivers/accgyro_mpu.c \
            drivers/accgyro_mpu6050.c \
			drivers/accgyro_mpu6500.c \
			drivers/accgyro_spi_mpu6500.c\
			drivers/cc2500.c\
			blackbox/blackbox.c \
            blackbox/blackbox_io.c \
			rx/frskyD_rx.c\

