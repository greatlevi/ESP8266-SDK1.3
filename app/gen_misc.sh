#!/bin/bash

echo "gen_misc.sh version 20151112"
echo "gen flash=4Mbytes"
#touch user/user_main.c

boot=new
app=1
spi_speed=2
spi_mode=0
spi_size_map=4

make COMPILE=gcc BOOT=$boot APP=$app SPI_SPEED=$spi_speed SPI_MODE=$spi_mode SPI_SIZE_MAP=$spi_size_map