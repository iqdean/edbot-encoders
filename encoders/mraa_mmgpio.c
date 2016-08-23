// 20150715  ike a dean <iad1046@gmail.com>

// To Build on ubilinux
// $ gcc -Wall -o mraammio mraa_mmgpio.c -lmraa

//REF1 :
//http://www.i-programmer.info/programming/hardware/
//       8770-exploring-edison-fast-memory-mapped-io.html 

//REF2 : MRAA to Arduino Pin Mappings
// https://learn.sparkfun.com/tutorials/
//         installing-libmraa-on-ubilinux-for-edison
// MRAA   Arduino   --- Pinmode -----
// Pin#   Pin#        0*        1
// 37     13        GPIO-40   SPI_CLK
// 38     11        GPIO-43   SPI_TxD
// 50     12        GPIO-42   SPI_RXD

// * - Pinmode 0  is the default

#include "mraa.h"
#include <stdio.h>
#include <unistd.h>

int main()
{
 mraa_init();

 char* board_name = mraa_get_platform_name();
 fprintf(stdout, "hello mraa\n Version: %s\n Running on %s\n", mraa_get_version(), board_name);

 mraa_gpio_context spiclk = mraa_gpio_init(13);
 
 mraa_gpio_dir(spiclk, MRAA_GPIO_OUT);  // set dir out
 mraa_gpio_mode(spiclk, MRAA_GPIO_PULLUP);
 mraa_gpio_use_mmaped(spiclk,1);        // use mmapped_io

 for (;;) {
  mraa_gpio_write(spiclk, 0);
  mraa_gpio_write(spiclk, 1);
 }

 return MRAA_SUCCESS;
}

