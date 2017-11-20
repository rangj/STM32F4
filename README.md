# STM32F4

# makefile
You can use command `make` to compile this project ,
MCU default is stm32f407vet6

you can run `./build.sh` within `STM32F4`, which generates  `blink.bin`.

if you want to download your pcb, you can install `JLink SEGGER` 

when you connect the JLink, follow this command to dowmload:

`loadbin xxx/STM32F4/blink.bin 0x08000000` ,if successed, you can see 
all flash is done. 

# app.tar.gz

app.tar.gz is SR2D firmware,this is for your reference only.

