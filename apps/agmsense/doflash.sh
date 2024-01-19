#!/bin/bash

arm-none-eabi-objcopy -O binary ./target/thumbv7m-none-eabi/release/agmsense ./target/thumbv7m-none-eabi/release/agmsense.bin
/Applications/SEGGER/JLink/JFlash.app/Contents/MacOS/JFlashExe -openprj../../SEGGER\ -\ FM3.jflash -open./target/thumbv7m-none-eabi/release/agmsense.bin,0x00000000 -auto -exit