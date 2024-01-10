#!/bin/bash

arm-none-eabi-objcopy -O binary ./target/thumbv7m-none-eabi/release/blinkenlight ./target/thumbv7m-none-eabi/release/blinkenlight.bin
/Applications/SEGGER/JLink/JFlash.app/Contents/MacOS/JFlashExe -openprj../../SEGGER\ -\ FM3.jflash -open./target/thumbv7m-none-eabi/release/blinkenlight.bin,0x00000000 -auto -exit
