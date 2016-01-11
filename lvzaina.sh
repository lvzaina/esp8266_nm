#!/bin/bash
make clean
echo "make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=2"
make COMPILE=gcc BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=2
cp ../bin/upgrade/user1.1024.new.2.bin /media/sf_GX/esp_rom/lvzaina.bin
sync
