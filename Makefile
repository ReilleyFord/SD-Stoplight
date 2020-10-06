SD-Stoplight: main.c
	avr-gcc -std=c99 -Wall -Os -DF_CPU=16000000 -mmcu=attiny804 -c main.c -o out/SD-Stoplight.o
	avr-gcc -std=c99 -Wall -Os -DF_CPU=16000000 -mmcu=attiny804 -o out/SD-Stoplight.elf out/SD-Stoplight.o
	avr-objcopy -j .text -j .data -O ihex out/SD-Stoplight.elf out/SD-Stoplight.hex
