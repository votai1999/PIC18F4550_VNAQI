#include <18F4550.h>
#device ADC=10
#use delay(crystal=20000000)
#use rs232(baud=9600,parity=N,xmit=PIN_B6,rcv=PIN_A5,bits=8,stream=PORT1)

