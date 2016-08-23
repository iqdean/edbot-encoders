// loopback test of bitbang edison spi i/f using fastio
// edspiloop.c

// To Build on ubilinux
// $ gcc -Wall -o edspiloop edspiloop.c -lmraa

/*
 * ref - ls7263 quad enc i/f :   galileo spi52.c
 * [2] Robogia 3axis encoder decode shield wires the X-axis ls7366 chipset to arduino gpio 10
 *     & all 3 ls7366 decoders are connected to SPI0 on the arduino ICSP connector.
 *     The Intel Galileo Gen1 board muxes arduino GPIO10 w SPI0 CS
 *     the idoitic hw ?or? sw (libmraa) doesn't allow
 *     galileo gpio 10 to be used as a regular GPIO (instead of SPI0 CS) when use of spi0 is enabled
 *     >> every serial exchange on spi0 is windowed by the spi0 cs, which shows up on gpio10 <<
 *
 *     WORKAROUND:
 *     1. on robogia, bend gpio pin10 up so it doesn't connect to galileo SPI0_CS on gpio10
 *        when the 2 boards are mated
 *     2. on robogio, jumper gpio7 to gpio10
 *     3. in galileo sw, use gpio7 instead of gpio10 to drive X-axis chip select
 */

/**********
ref2 - robogia bitbang spi gpio pin i/f:  galileo arduino 
2014\SWDEV\Intel-Galileo-Board\arduino-sketchs\bitBangSpi\bitbangspi_ino

const int SS_pin = 9;    //#define GPIO_FAST_IO9   GPIO_FAST_ID_QUARK_NC_RW(0x04)
const int SCK_pin = 13;  //#define GPIO_FAST_IO13  GPIO_FAST_ID_QUARK_NC_RW(0x20)  \  SPI CLK
const int MISO_pin = 12; //#define GPIO_FAST_IO12  GPIO_FAST_ID_QUARK_SC(0x80)      > SPI MISO
const int MOSI_pin = 11; //#define GPIO_FAST_IO11  GPIO_FAST_ID_QUARK_NC_RW(0x08)  /  SPI MOSI
**********/

/*
For this loopback test
- do not connect the robogia board to edison
- jumper MOSI_pin IO11 to MISO_pin IO12 @ 6-pin spi header on the edison arduino board
- monitor CS, SCK, MOSI, MISO on scope while running the loopback test
 */

/* ref 3  https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
   spi bus timings... data is clk'd in and out on the same edge 
 */

#include "mraa.h"
#include <stdio.h>
#include <unistd.h>

#define SCK_GPIO   13
#define MISO_GPIO  12
#define MOSI_GPIO  11
#define CS_GPIO     6

unsigned char spi_xcv_byte(unsigned char txd_byte,
			   mraa_gpio_context CS_pin,
			   mraa_gpio_context SCK_pin,
			   mraa_gpio_context MOSI_pin,
			   mraa_gpio_context MISO_pin )
 {
 	int i = 0;

	unsigned char txd_bit;
	unsigned char txd_mask = 1;

	unsigned char rxd_bit;
	unsigned char rcv_byte = 0;

	mraa_gpio_write(CS_pin, 0);   // CS  1->0  Select
	mraa_gpio_write(SCK_pin,0);   // Clk 1->0  then drive clk low

 	for(i=0; i<8; i++)
 	{
		txd_bit = (txd_byte >> (7 - i) & txd_mask);
		mraa_gpio_write(MOSI_pin, txd_bit); 
		mraa_gpio_write(SCK_pin,1);          // Clk 0->1
		rxd_bit = mraa_gpio_read(MISO_pin);
		mraa_gpio_write(SCK_pin,0);          // Clk 1->0
		rcv_byte = (rcv_byte << 1)|rxd_bit;
	}

	mraa_gpio_write(CS_pin, 1);   // CS  0->1  Deselect
        mraa_gpio_write(SCK_pin,1);   // Clk 0->1  

	return 	rcv_byte;
 }


int main()
{
 mraa_init();

 char* board_name = mraa_get_platform_name();
 fprintf(stdout, "hello mraa\n Version: %s\n Running on %s\n", mraa_get_version(), board_name);

 // 1. cfg Arduino Pin IO13 as SCK 

 mraa_gpio_context SCK_pin = mraa_gpio_init(SCK_GPIO);

 mraa_gpio_dir(SCK_pin, MRAA_GPIO_OUT);      // set dir OUT
 mraa_gpio_mode(SCK_pin, MRAA_GPIO_PULLUP);
 mraa_gpio_use_mmaped(SCK_pin,1);            // use mmapped_io 

 // 2. cfg Arduino Pin IO12 as MISO 

 mraa_gpio_context MISO_pin = mraa_gpio_init(MISO_GPIO);

 mraa_gpio_dir(MISO_pin, MRAA_GPIO_IN);      // set dir IN
 mraa_gpio_use_mmaped(MISO_pin,1);           // use mmapped_io 

 // 3. cfg Arduino Pin IO11 as MOSI 

 mraa_gpio_context MOSI_pin = mraa_gpio_init(MOSI_GPIO);

 mraa_gpio_dir(MOSI_pin, MRAA_GPIO_OUT);      // set dir OUT
 mraa_gpio_mode(MOSI_pin, MRAA_GPIO_PULLUP);
 mraa_gpio_use_mmaped(MOSI_pin,1);            // use mmapped_io 

 // 4. cfg Arduino Pin IO9 as SS_pin 

 mraa_gpio_context CS_pin = mraa_gpio_init(CS_GPIO);

 mraa_gpio_dir(CS_pin, MRAA_GPIO_OUT);      // set dir OUT
 mraa_gpio_mode(CS_pin, MRAA_GPIO_PULLUP);
 mraa_gpio_use_mmaped(CS_pin,1);            // use mmapped_io 

 // 5. Initialize SPI I/F to LS7263

 mraa_gpio_write(SCK_pin,1);
 mraa_gpio_write(CS_pin, 1);
 mraa_gpio_write(MOSI_pin,1);

 unsigned char spi_byte_to_xmit = 0x5f;
 unsigned char spi_byte_rcvd = 0x00;

 printf("looping on spi_xcv_byte(%x)...\n",spi_byte_to_xmit);
 printf("CHK SPI I/F pins wiggling? : IO13-SCK IO12-MIS0 IO11-MOSI IO9-CS \n");

	 for (;;){

	   spi_byte_rcvd = spi_xcv_byte(spi_byte_to_xmit,
					CS_pin,
					SCK_pin,
					MOSI_pin,
					MISO_pin);

	   printf("spi_xcv_byte(txbyte) returned: %x\n", spi_byte_rcvd);

	 }

 return MRAA_SUCCESS;
}
