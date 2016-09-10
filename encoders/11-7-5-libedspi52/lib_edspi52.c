/*
 * lib_edspi42.c - Refactor edspi52.c to create share lib that can
 *                 be called from python to read motor encoders
 * WHY? 
 * ROS Node to do odometery based on wheel encoders will be written in python
 *
 * REF: best-cytpes-ref-yet-linux   myTestLib.c & mypython.py
 *      How to write a shared library in C
 *      How to call functions exported by shared library from python
 *
 * To build shared library on Linux:
 *
 * gcc -c -Wall -Werror -fpic lib_edspi52.c -lrt -lmraa   < uses librt and libmraa 
 * gcc -shared  -o libedspi52.so lib_edspi52.o            
 * 
 * To see functions exported by libmydll.so, use nm:
 * $ nm -C -D -g libmydll.so 
 *
 */

#ifdef _WIN32
#  define EXPORTIT __declspec( dllexport )
#else
#  define EXPORTIT
#endif

#include "/usr/local/include/mraa.h"
#include "/usr/local/include/mraa/gpio.h"
#include "/usr/local/include/mraa/spi.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>
#include <time.h>
#include <ncurses.h>

/*
 * LS7366 SPI-based 32bit Quadrature Encoder Decoder
 * register r/w commands
 * http://www.lsicsi.com/pdfs/Data_Sheets/LS7366R.pdf
 *
 *---- IR REG -------------  1 INSTRUCTION BYTE (1st byte transfered to the LS7366)
b7 b6 b5 b4 b3 b2 b1 b0
-op-  --reg--  x  x  x

b7 b6
0  0  CLR Reg
0  1  RD  Reg
1  0  WR  Reg
1  1  Load Reg

      b5 b4 b3
      0  0  0 Select None
      0  0  1 Select MDR0
      0  1  0 Select MDR1
      0  1  1 Select DTR
      1  0  0 Select CNTR
      1  0  1 Select OTR
      1  1  0 Select STR
      0  0  0 Select None

          7 6 5 4 3 2 1 0
RD_MDR0 = 0 1 0 0 1 x x x = 0x48h
WR_MDR0 = 1 0 0 0 1 x x x = 0x88h

RD_MDR1 = 0 1 0 1 0 x x x = 0x50h
WR_MDR1 = 1 0 0 1 0 x x x = 0x90h

RD_STR  = 0 1 1 1 0 x x x = 0x70h
RD_CNTR = 0 1 1 0 0 x x x = 0x60h    xfr CNTR to OTR, output OTR onto TXD (MISO)

CLR_CNTR= 0 0 1 0 0 x x x = 0x20h    clr CNTR

---- STR REG --------------  2 STATUS REG
b7 b6 b5 b4 b3 b2 b1 b0
            |
            \- Count Enable Status (0 = disabled, 1=enabled)

----- MDR0 ----------------  3 MODE REG 0 = 0x03h    PowerOn = 0x00h
b7   b6 b5 b4  b3 b2 b1 b0
clk  -index-   count mode
                     0  0 - non-quad (A=clk, B=Dir)
                     1  1 - x4 quad count mode (4 counts/quad cycle)
               0  0       - free-running count mode
     0  0   0             - disable index
 0                        - filter clock div-by-1

----- MDR1 ----------------   4 MODE REG 1 = 0x00h    PowerOn = 0x00h
b7 b6 b5  b4   b3  b2  b1 b0
                       0  0 - 4 Byte 32bit Counter
                       0  1 - 3 Byte 24bit Counter
                       1  0 - 2 Byte 16bit Counter
                       1  1 - 1 Byte  8bit Counter
                   0 - Enable Counting
                   1 - Disable Counting
               na - not used
CY BW CMP IDX - Flags

----- CNTR -----------------
8,16,24, or 32bit up/down counter which counts the up/down pulses
resulting from the quadrature clocks applied at A & B inputs.

----- Transmission Cycle ----

--
  \__  Transmission cycle initiated by H->L on SS/ (chip select)

->  IR 1st byte xfered is always the Instruction IR Byte

<-> data 2nd to 5th byte is data (from host to device or visa versa)
    MSBit 1st (d7 d6 d5 ... d2 d1 d0) & MSByte 1st (32:24, 23:16, ... 7:0)

   --
__/    Transmission cycle is terminated by L->H on SS/ (chip select)
 
 */

#define RD_MDR0 0x48
#define WR_MDR0 0x88
#define RD_MDR1 0x50
#define WR_MDR1 0x90
#define RD_STR  0x70
#define RD_CNTR 0x60	// xfr CNTR to OTR, output OTR onto TXD (MISO)
/*
 * [1] you can change the wheel encoder resolution (CPR - Counts Per Revolution)
 *     by changning the value of MDR_qxx in the init code
 */
#define MDR0_q4x 0x03	// 4x quadrature decode	[1]
#define MDR0_q2x 0x02	// 2x quadrature decode [1]
#define MDR0_q1x 0x01	// 1x quadrature decode [1]
#define MDR0_q0x 0x00   // non-quad (A=clk, B=Dir)

#define MDR1_en32 0x00	// enable counting w 32bit counter

// Update to match HW Config used for AxisEncoderShield4.ino
// chg'd to move all 3 gpio's into the same cluster so we could use r/m/w fastgpio macros
// int csX = 5;
// int csY = 9;
// int csZ = 6;

// Update for use with Edison Arduino Board
//int csX = 7;   // jmp pin 7 to pin 10 on robogia
int csX = 6;     // jmp pin 6 to pin 10, if use pin 7 on edison, wifi breaks
int csY = 9;
int csZ = 8;


#define SCK_GPIO   13
#define MISO_GPIO  12
#define MOSI_GPIO  11

//mraa_spi_context spi;
mraa_gpio_context SCK_pin;
mraa_gpio_context MISO_pin;
mraa_gpio_context MOSI_pin;

mraa_gpio_context Xcs;
mraa_gpio_context Ycs;
mraa_gpio_context Zcs;

int set_high = 1;
int set_low = 0;

uint8_t txb;
uint8_t rxb;

typedef struct {
  uint32_t x_enc_cnt;     // x-axis encoder count
  uint32_t x_ts_sec;      // x-axis timestamp (sec:nsec)
  uint32_t x_ts_ns;
  uint32_t y_enc_cnt;     // y-axis encoder count
  uint32_t y_ts_sec;      // y-axis timestamp (sec:nsec)
  uint32_t y_ts_ns;
} mtrEnc;

/* ------- function defs ------*/

//                         cspin = Xcs:Ycs:Zcs
mraa_result_t gpio_set(mraa_gpio_context csPin, int level) {
	return mraa_gpio_write(csPin, level);
}

/*
 * init gpio pins to be used for chip selects & spi i/f
 */
void initGpio() {

        SCK_pin = mraa_gpio_init(SCK_GPIO);
	mraa_gpio_dir(SCK_pin, MRAA_GPIO_OUT);      // set dir OUT
	mraa_gpio_mode(SCK_pin, MRAA_GPIO_PULLUP);
	mraa_gpio_use_mmaped(SCK_pin,1);            // use mmapped_io 

	MISO_pin = mraa_gpio_init(MISO_GPIO);
	mraa_gpio_dir(MISO_pin, MRAA_GPIO_IN);      // set dir IN
	mraa_gpio_use_mmaped(MISO_pin,1);           // use mmapped_io 

	MOSI_pin = mraa_gpio_init(MOSI_GPIO);
	mraa_gpio_dir(MOSI_pin, MRAA_GPIO_OUT);      // set dir OUT
	mraa_gpio_mode(MOSI_pin, MRAA_GPIO_PULLUP);
	mraa_gpio_use_mmaped(MOSI_pin,1);            // use mmapped_io 

        Xcs = mraa_gpio_init(csX);
	Ycs = mraa_gpio_init(csY);
	Zcs = mraa_gpio_init(csZ);

	mraa_gpio_dir(Xcs, MRAA_GPIO_OUT);  // set directioon
	mraa_gpio_dir(Ycs, MRAA_GPIO_OUT);
	mraa_gpio_dir(Zcs, MRAA_GPIO_OUT);

	mraa_gpio_mode(Xcs, MRAA_GPIO_PULLUP); // set output pin cfg
	mraa_gpio_mode(Ycs, MRAA_GPIO_PULLUP); 
	mraa_gpio_mode(Zcs, MRAA_GPIO_PULLUP);

	mraa_gpio_use_mmaped(Xcs,1);            // use mmapped_io 
	mraa_gpio_use_mmaped(Ycs,1);            // use mmapped_io 
	mraa_gpio_use_mmaped(Zcs,1);            // use mmapped_io 

	//gpio_set(SCK_pin, set_high);
	gpio_set(SCK_pin, set_low);  // ls7366 ds: sck idle state = 0
	gpio_set(MOSI_pin, set_high);
	gpio_set(Xcs, set_high);
	gpio_set(Ycs, set_high);
	gpio_set(Zcs, set_high);

	return;
}

uint8_t spi_xcv_byte(uint8_t txd_byte)
 {
 	int i = 0;

	unsigned char txd_bit;
	unsigned char txd_mask = 1;

	unsigned char rxd_bit;
	unsigned char rcv_byte = 0;

	// mraa_gpio_write(SCK_pin,0);   // Clk 1->0  then drive clk low

 	for(i=0; i<8; i++)
 	{
		txd_bit = (txd_byte >> (7 - i) & txd_mask);
		mraa_gpio_write(MOSI_pin, txd_bit); 
		mraa_gpio_write(SCK_pin,1);          // Clk 0->1
		rxd_bit = mraa_gpio_read(MISO_pin);
		mraa_gpio_write(SCK_pin,0);          // Clk 1->0
		rcv_byte = (rcv_byte << 1)|rxd_bit;
	}

        // mraa_gpio_write(SCK_pin,1);   // Clk 0->1  

	return 	rcv_byte;
 }

int rd7366SpiReg(mraa_gpio_context csPin, int cmd){		// 2 byte operation
	gpio_set(csPin, set_low);	//rw transaction initiated on falling edge of chip select
	txb = cmd;

        rxb = spi_xcv_byte(txb);

	rxb = spi_xcv_byte(txb);
	gpio_set(csPin, set_high);
	return rxb;
}

void wr7366SpiReg(mraa_gpio_context csPin, int cmd, int val){ 	// 2 byte operation
	gpio_set(csPin, set_low);
	txb = cmd;

	rxb = spi_xcv_byte(txb);
	txb = val;
	
	rxb = spi_xcv_byte(txb);
	gpio_set(csPin, set_high);
	return;
}

void init7366(){
    gpio_set(Xcs, set_high);
    gpio_set(Ycs, set_high);
    gpio_set(Zcs, set_high);		       // set all chip selects = 1

    // initialize X-Axis encoder decoder
    wr7366SpiReg(Xcs, WR_MDR0, MDR0_q4x);	// set X axis MDR0 = 03
    wr7366SpiReg(Xcs, WR_MDR1, MDR1_en32);	// set X axis MDR1 = 00

    // initialize Y-Axis encoder decoder
    wr7366SpiReg(Ycs, WR_MDR0, MDR0_q4x);	// set Z axis MDR0 = 03

    wr7366SpiReg(Ycs, WR_MDR1, MDR1_en32);	// set Z axis MDR1 = 00

    // initialize Z-Axis encoder decoder
    wr7366SpiReg(Zcs, WR_MDR0, MDR0_q4x);	// set Z axis MDR0 = 03

    wr7366SpiReg(Zcs, WR_MDR1, MDR1_en32);	// set Z axis MDR1 = 00

}

int getEncoderCount(mraa_gpio_context csAxis){	// 5 byte op
	uint8_t bv3, bv2, bv1, bv0;		// bv3 = ms_byte, bv0 = ls_byte
	int result;

	gpio_set(csAxis, set_low);		// select encoder

	txb = RD_CNTR;

	rxb = spi_xcv_byte(txb);	// 1 txb = RD_CNTR  rxb = junk
	bv3 = spi_xcv_byte(txb);	// 2 txb = junk     rxb = bv3	msb
	bv2 = spi_xcv_byte(txb);	// 3 txb = junk     rxb = bv2
	bv1 = spi_xcv_byte(txb);	// 4 txb = junk     rxb = bv1
	bv0 = spi_xcv_byte(txb);	// 5 txb = junk     rxb = bv0	lsb

	gpio_set(csAxis, set_high);		// deselect encoder

	result = (bv3<<24)+(bv2<<16)+(bv1<<8)+(bv0);

	return result;
}

void nsleep(long us)
{
    struct timespec wait;

    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}

/* ---- main entry point ---- */
/* TODO: replace main() with

      OUT               IN
   char * edspi52_init(void)       - initializes stuff, call before using the lib
          IN -> na
          OUT <- mraa_get_version() OR 'init ERROR'

      int edspi52_deinit(void)     - de-initializes stuff, call when done using the lib
          IN -> na
          OUT <- 0 success  -1 error

   mtrEnc getXYEncCount(void)      - read XY enc counts w timestamps 
          IN -> na
          OUT <- mtrEnc struct, returns 6 double Words over the return stack
                Error condition is signaled with (x_ts_sec == x_ts_ns)

Cleaner way to do error reporting is:
1. Caller provides ptr to memory buffer using malloc
2. library functions fill in the buffer with return values using memcpy
   & Return ERROR Codes

int func( mtrEnc * )

But Then, u have to figure out how to do memcpy operation on the python side:

REF:
http://stackoverflow.com/questions/9215729/adding-data-to-a-c-buffer-passed-to-a-python-ctypes-callback
> ctypes has memmove function which wraps libc memcpy
> http://dabeaz.blogspot.com/2009/08/python-binary-io-handling.html
> http://stackoverflow.com/questions/1470343/python-ctypes-copying-structures-contents

*********************/

/***** NEW LIBRARY CODE ******/
// TODO: add error checking and return success OR error code

EXPORTIT int edspi52_init(void) {
  mraa_init();
  //printf("mraa version: %s \n", mraa_get_version());
  initGpio();
  init7366();
  return 0;
}

EXPORTIT int edspi52_deinit(void) {
  mraa_deinit();
  return 0;
}

EXPORTIT mtrEnc getXYEncCount(void) {

  int x_axis, y_axis;
  struct timespec xt;
  struct timespec yt;
  mtrEnc U;

  x_axis = getEncoderCount(Xcs);
  clock_gettime(CLOCK_REALTIME, &xt);

  y_axis = getEncoderCount(Ycs);
  clock_gettime(CLOCK_REALTIME, &yt);

  U.x_enc_cnt = x_axis;
  U.x_ts_sec  = xt.tv_sec;
  U.x_ts_ns   = xt.tv_nsec;

  U.y_enc_cnt = y_axis;
  U.y_ts_sec  = yt.tv_sec;
  U.y_ts_ns   = yt.tv_nsec;
/***
  printf("X axis count\t%8X\n", x_axis );
  printf("X time stamp\t%d %d\n", U.x_ts_sec, U.x_ts_ns );
  printf("Y axis count\t%8X\n", y_axis );
  printf("Y time stamp\t%d %d\n", U.y_ts_sec, U.y_ts_ns );
***/
  return U;
}



