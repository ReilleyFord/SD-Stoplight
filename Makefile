Polyphemus: main.c
	avr-gcc -Wall -Os -DF_CPU=8000000 -mmcu=attiny804 -c SD-Stoplight.c -o main.hex
