#!/bin/bash

arm-none-eabi-objcopy -O binary ./target/thumbv7m-none-eabi/release/sermon ./target/thumbv7m-none-eabi/release/sermon.bin
/Applications/SEGGER/JLink/JFlash.app/Contents/MacOS/JFlashExe -openprj../../SEGGER\ -\ FM3.jflash -open./target/thumbv7m-none-eabi/release/sermon.bin,0x00000000 -auto -exit