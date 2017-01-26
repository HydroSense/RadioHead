// RasPi.h
//
// Routines for implementing RadioHead on Raspberry Pi
// ported to use WiringPi to enable interrupts by Alan Marchiori
// Modified by the file contributed by Mike Poublon 

#ifndef RASPI_h
#define RASPI_h

//#include <bcm2835.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef unsigned char byte;

#ifndef RISING
 #define RISING INT_EDGE_RISING
#endif
#ifndef FALLING
 #define FALLING INT_EDGE_FALLING
#endif

/*
#ifndef NULL
  #define NULL 0
#endif

#ifndef OUTPUT
  #define OUTPUT BCM2835_GPIO_FSEL_OUTP
#endif

#ifndef INPUT
  #define INPUT BCM2835_GPIO_FSEL_INPT
#endif
*/

class HardwareSPI
{
  public:
    HardwareSPI(int channel);
    byte transfer(byte _data);
    // SPI Configuration methods
    void begin(); // Default
    void begin(int);
    void end();    
  private:
    int _fd;
    int _chan;
};

//extern HardwareSPI HardwareSPI;

class SerialSimulator
{
  public:
    #define DEC 10
    #define HEX 16
    #define OCT 8
    #define BIN 2

    // TODO: move these from being inlined
    static void begin(int baud);
    static size_t println(const char* s);
    static size_t print(const char* s);
    static size_t print(unsigned int n, int base = DEC);
    static size_t print(char ch);
    static size_t println(char ch);
    static size_t print(unsigned char ch, int base = DEC);
    static size_t println(unsigned char ch, int base = DEC);
};

extern SerialSimulator Serial;

//void RasPiSetup();

//void pinMode(unsigned char pin, unsigned char mode);

//void digitalWrite(unsigned char pin, unsigned char value);

//unsigned long millis();

//void delay (unsigned long delay);

void attachInterrupt(int intNum, void (*function)(void), int mode);

long random(long min, long max);

#endif /* RASPI_h */
