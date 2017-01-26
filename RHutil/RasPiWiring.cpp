// RasPi.cpp
//
// Routines for implementing RadioHead on Raspberry Pi
// using BCM2835 library for GPIO
//
// Contributed by Mike Poublon and used with permission


#include <RadioHead.h>

#if (RH_PLATFORM == RH_PLATFORM_RASPI)
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include "RasPiWiring.h"

//Initialize the values for sanity
timeval RHStartTime;

HardwareSPI::HardwareSPI(int channel)
{
  _chan = channel;
}
void HardwareSPI::begin()
{
  //Set SPI Defaults
  //uint16_t divider = BCM2835_SPI_CLOCK_DIVIDER_256;
  //uint8_t bitorder = BCM2835_SPI_BIT_ORDER_MSBFIRST;
  //uint8_t datamode = BCM2835_SPI_MODE0;

  //begin(divider, bitorder, datamode);
  begin(1e6);
}

//void HardwareSPI::begin(uint16_t divider, uint8_t bitOrder, uint8_t dataMode)
void HardwareSPI::begin(int speed)
{
  printf("starting WiringPi SPI channel %d\n", _chan);
  _fd = wiringPiSPISetup(_chan, speed);

  //Initialize a timestamp for millis calculation
  gettimeofday(&RHStartTime, NULL);
}

void HardwareSPI::end()
{
  //End the SPI
  close(_fd);
}
/*
void HardwareSPI::setBitOrder(uint8_t bitOrder)
{
  //Set the SPI bit Order
  bcm2835_spi_setBitOrder(bitOrder);
}

void HardwareSPI::setDataMode(uint8_t mode)
{
  //Set SPI data mode
  bcm2835_spi_setDataMode(mode);
}

void HardwareSPI::setClockDivider(uint16_t rate)
{
  //Set SPI clock divider
  bcm2835_spi_setClockDivider(rate);
}
*/
byte HardwareSPI::transfer(byte _data)
{

  /*
  //Set which CS pin to use for next transfers
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  //Transfer 1 byte
  byte data;
  data = bcm2835_spi_transfer((uint8_t)_data);
  return data;
  */

  // copy so we don't change the argument value
  byte buf = _data;

  wiringPiSPIDataRW(_chan, &buf, 1);

  return buf;
}

/* wiringPi does this
void pinMode(unsigned char pin, unsigned char mode)
{
  if (mode == OUTPUT)
  {
    bcm2835_gpio_fsel(pin,BCM2835_GPIO_FSEL_OUTP);
  }
  else
  {
    bcm2835_gpio_fsel(pin,BCM2835_GPIO_FSEL_INPT);
  }
}

void digitalWrite(unsigned char pin, unsigned char value)
{
  bcm2835_gpio_write(pin,value);
}



unsigned long millis()
{
  //Declare a variable to store current time
  struct timeval RHCurrentTime;
  //Get current time
  gettimeofday(&RHCurrentTime,NULL);
  //Calculate the difference between our start time and the end time
  unsigned long difference = ((RHCurrentTime.tv_sec - RHStartTime.tv_sec) * 1000);
  difference += ((RHCurrentTime.tv_usec - RHStartTime.tv_usec)/1000);
  //Return the calculated value
  return difference;
}

void delay (unsigned long ms)
{
  //Implement Delay function
  struct timespec ts;
  ts.tv_sec=0;
  ts.tv_nsec=(ms * 1000);
  nanosleep(&ts,&ts);
}
*/


/* map arduino style attachInterrupt to wiringPi style pin */
void attachInterrupt(int intNum, void (*function)(void), int mode)
{
  printf("attachInt (%d, %p, %d)\n", intNum, function, mode);
  wiringPiISR(intNum, mode, function);
}

long random(long min, long max)
{
  long diff = max - min;
  long ret = diff * rand() + min;
  return ret;
}

void SerialSimulator::begin(int baud)
{
  //No implementation neccesary - Serial emulation on Linux = standard console
  //
  //Initialize a timestamp for millis calculation - we do this here as well in case SPI
  //isn't used for some reason
  gettimeofday(&RHStartTime, NULL);
}

size_t SerialSimulator::println(const char* s)
{
  print(s);
  printf("\n");
}

size_t SerialSimulator::print(const char* s)
{
  printf(s);
}

size_t SerialSimulator::print(unsigned int n, int base)
{
  if (base == DEC)
    printf("%d", n);
  else if (base == HEX)
    printf("%02x", n);
  else if (base == OCT)
    printf("%o", n);
  // TODO: BIN
}

size_t SerialSimulator::print(char ch)
{
  printf("%c", ch);
}

size_t SerialSimulator::println(char ch)
{
  printf("%c\n", ch);
}

size_t SerialSimulator::print(unsigned char ch, int base)
{
  return print((unsigned int)ch, base);
}

size_t SerialSimulator::println(unsigned char ch, int base)
{
  print((unsigned int)ch, base);
  printf("\n");
}


#endif /* (RH_PLATFORM == RH_PLATFORM_RASPI) */
