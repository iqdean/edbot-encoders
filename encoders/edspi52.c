/*  <ike.a.dean> iad1046@gmail.com
 *  edspi52.c - Read 32bit quadrature encoder counter from spi-based LS7366 shield
 *
 *  To Build on iot-devkit linux image:
 *  $ gcc -Wall -o edspi52 edspi52.c -lmraa -lncurses
 *
 *  Revision History
 *  12-14-15  edspi52.c 1st port of galileo spi52.c to edison
 *  1. Replace slow sysfs-based spi i/f  
 *     > rxb = mraa_spi_write(spi, txb)
 *     with fast memmapped io spi i/f    
 *     > rxb = spi_xcv_byte(txb, SCK_pin, MOSI_pin, MISO_pin)

    12-16-15 benchmark edspi52.c timings without robogia hooked up

          mraa version: v0.8.1-21-g5cf54fd

          Set cs[X Y Z] = 1

          Xcs WR_MDR0: 88
          Xcs RD_MDR0: FF
          Xcs WR_MDR1: 90
          Xcs RD_MDR0: FF

          Ycs WR_MDR0: 88
          Ycs RD_MDR0: FF
          Ycs WR_MDR1: 90
          Ycs RD_MDR0: FF

          Zcs WR_MDR0: 88
          Zcs RD_MDR0: FF
          Zcs WR_MDR1: 90
          Zcs RD_MDR0: FF
                                 edspi52        spi52
          X axis count         0  752     vs    7000 6600
          Y axis count         0  692           9400 1200
          Z axis count         0  693           9500 1200

 Hmm, after running edspi52, ssh over wifi becomes unusable till u reboot or pwr cycle ???

 See if serial console has same issue?  - ok, console works fine after running

          X axis count         0  758
          Y axis count         0  697
          Z axis count         0  693

 * 12-25-15  update Robogia CS pin to use pin 6 to avoid 
 *           Wifi stops working, becomes unusable as soon as mraa tries to use GPIO48 (pin 7)

 * 12-26-15  Currently, SPI SCK IDLE state is 1, and basic reg wr/rd ops to ls7366 r failing
 *           Per ls7366 datasheet, SPI SCK IDLE state must = 0, NOT 1
 *           update initGPIO() and spi_xcv_byte() to make SPI SCK IDLE state = 0 INSTEAD of 1 

 */

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
 *
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


/* ------- Globals ------------ */

/*
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

//int csX = 7;	// X axis LS7366r CS  gpio 7  [2]
//int csY = 9;    // Y axis LS7366r CS  gpio 9
//int csZ = 8;    // Z axis LS7366r CS  gpio 8

// Update spi52.c for Gen2 board:

//int csX = 10;	// X axis LS7366r CS  gpio 10
//int csY = 9;    // Y axis LS7366r CS  gpio 9
//int csZ = 8;    // Z axis LS7366r CS  gpio 8

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

mraa_uart_context uart;

int set_high = 1;
int set_low = 0;

uint8_t txb;
uint8_t rxb;

WINDOW* window;			// pointer to ncurses window

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
	//mraa_gpio_mode(MISO_pin, MRAA_GPIO_PULLUP);
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

	gpio_set(Xcs, set_high);
	gpio_set(Ycs, set_high);
	gpio_set(Zcs, set_high);
	gpio_set(SCK_pin, set_low);  // ls7366 ds: sck idle state = 0
	gpio_set(MOSI_pin, set_high);

	uart = mraa_uart_init(0);

	return;
}

uint8_t spi_xcv_byte(uint8_t txd_byte)
 {
 	int i = 0;

	unsigned char txd_bit;
	unsigned char txd_mask = 1;

	unsigned char rxd_bit;
	unsigned char rcv_byte = 0;

	//mraa_gpio_write(SCK_pin,0);   // Clk IDLE = 0

 	for(i=0; i<8; i++)
 	{
		txd_bit = (txd_byte >> (7 - i) & txd_mask);
		mraa_gpio_write(MOSI_pin, txd_bit);  // setup the data
		mraa_gpio_write(SCK_pin,1);          // Clk 0->1  
		rxd_bit = mraa_gpio_read(MISO_pin);
		mraa_gpio_write(SCK_pin,0);          // Clk 1->0
		rcv_byte = (rcv_byte << 1)|rxd_bit;
	}

        // mraa_gpio_write(SCK_pin,1);   // Clk IDLE = 0 

	return 	rcv_byte;
 }

int rd7366SpiReg(mraa_gpio_context csPin, int cmd){		// 2 byte operation
	gpio_set(csPin, set_low);	//rw transaction initiated on falling edge of chip select
	txb = cmd;
	//rxb = mraa_spi_write(spi,txb);  //1st txb = reg_r_cmd rxb = junk
        rxb = spi_xcv_byte(txb);
	//rxb = mraa_spi_write(spi,txb);  //2nd txb = junk      rxb = r_reg_value
	rxb = spi_xcv_byte(txb);
	gpio_set(csPin, set_high);
	return rxb;
}

void wr7366SpiReg(mraa_gpio_context csPin, int cmd, int val){ 	// 2 byte operation
	gpio_set(csPin, set_low);
	txb = cmd;
	//rxb = mraa_spi_write(spi,txb);	//1st txb = reg_w_cmd rxb = junk
	rxb = spi_xcv_byte(txb);
	txb = val;
	//rxb = mraa_spi_write(spi,txb);	//2nd txb = w_reg_val rxb = junk
	rxb = spi_xcv_byte(txb);
	gpio_set(csPin, set_high);
	return;
}

void init7366(){
    gpio_set(Xcs, set_high);
    gpio_set(Ycs, set_high);
    gpio_set(Zcs, set_high);			       // set all chip selects = 1
    mvwprintw(window, 3, 10,"Set cs[X Y Z] = 1 \n");

    // initialize X-Axis encoder decoder
    wr7366SpiReg(Xcs, WR_MDR0, MDR0_q4x);	// set X axis MDR0 = 03
    mvwprintw(window, 5, 10,"Xcs WR_MDR0: %2x\t", WR_MDR0);
    mvwprintw(window, 6, 10,"Xcs RD_MDR0: %2X \n", rd7366SpiReg(Xcs, RD_MDR0));

	wr7366SpiReg(Xcs, WR_MDR1, MDR1_en32);	// set X axis MDR1 = 00
	mvwprintw(window, 7, 10,"Xcs WR_MDR1: %2x\t", WR_MDR1);
	mvwprintw(window, 8, 10,"Xcs RD_MDR0: %2X \n", rd7366SpiReg(Xcs, RD_MDR1));

    // initialize Y-Axis encoder decoder
    wr7366SpiReg(Ycs, WR_MDR0, MDR0_q4x);	// set Z axis MDR0 = 03
    mvwprintw(window, 10, 10,"Ycs WR_MDR0: %2x\t", WR_MDR0);
    mvwprintw(window, 11, 10,"Ycs RD_MDR0: %2X \n", rd7366SpiReg(Ycs, RD_MDR0));

	wr7366SpiReg(Ycs, WR_MDR1, MDR1_en32);	// set Z axis MDR1 = 00
	mvwprintw(window, 12, 10,"Ycs WR_MDR1: %2x\t", WR_MDR1);
	mvwprintw(window, 13, 10,"Ycs RD_MDR0: %2X \n", rd7366SpiReg(Ycs, RD_MDR1));

    // initialize Z-Axis encoder decoder
    wr7366SpiReg(Zcs, WR_MDR0, MDR0_q4x);	// set Z axis MDR0 = 03
    mvwprintw(window, 15, 10,"Zcs WR_MDR0: %2x\t", WR_MDR0);
    mvwprintw(window, 16, 10,"Zcs RD_MDR0: %2X \n", rd7366SpiReg(Zcs, RD_MDR0));

	wr7366SpiReg(Zcs, WR_MDR1, MDR1_en32);	// set Z axis MDR1 = 00
	mvwprintw(window, 17, 10,"Zcs WR_MDR1: %2x\t", WR_MDR1);
	mvwprintw(window, 18, 10,"Zcs RD_MDR0: %2X \n", rd7366SpiReg(Zcs, RD_MDR1));
}

int getEncoderCount(mraa_gpio_context csAxis){	// 5 byte op
	uint8_t bv3, bv2, bv1, bv0;		// bv3 = ms_byte, bv0 = ls_byte
	int result;

	gpio_set(csAxis, set_low);		// select encoder

	txb = RD_CNTR;

	/*
	rxb = mraa_spi_write(spi,txb);	// 1 txb = RD_CNTR  rxb = junk
	bv3 = mraa_spi_write(spi,txb);	// 2 txb = junk     rxb = bv3	msb
	bv2 = mraa_spi_write(spi,txb);	// 3 txb = junk     rxb = bv2
	bv1 = mraa_spi_write(spi,txb);	// 4 txb = junk     rxb = bv1
	bv0 = mraa_spi_write(spi,txb);	// 5 txb = junk     rxb = bv0	lsb
	*/

	rxb = spi_xcv_byte(txb);	// 1 txb = RD_CNTR  rxb = junk
	bv3 = spi_xcv_byte(txb);	// 2 txb = junk     rxb = bv3	msb
	bv2 = spi_xcv_byte(txb);	// 3 txb = junk     rxb = bv2
	bv1 = spi_xcv_byte(txb);	// 4 txb = junk     rxb = bv1
	bv0 = spi_xcv_byte(txb);	// 5 txb = junk     rxb = bv0	lsb

	gpio_set(csAxis, set_high);		// deselect encoder

	result = (bv3<<24)+(bv2<<16)+(bv1<<8)+(bv0);

	return result;
}
// struct timespec (units: nanoseconds)  is different than struct timeval (units: microseconds)
// ===============                                         ==============
// REF: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
//
/*
The GNU C Library provides two data types specifically for representing an elapsed time. 
They are used by various GNU C Library functions, and you can use them for your own purposes too. 
They’re exactly the same except that one has a resolution in microseconds, and the other,
newer one, is in nanoseconds.

Data Type: struct timeval
The struct timeval structure represents an elapsed time. 
It is declared in sys/time.h and has the following members:

    long int tv_sec
    This represents the number of whole seconds of elapsed time.

    long int tv_usec
    This is the rest of the elapsed time (a fraction of a second), 
    represented as the number of microseconds. It is always less than one million.

Data Type: struct timespec
The struct timespec structure represents an elapsed time. 
It is declared in time.h and has the following members:

long int tv_sec
This represents the number of whole seconds of elapsed time.

long int tv_nsec
This is the rest of the elapsed time (a fraction of a second), represented as the number 
of nanoseconds. It is always less than one billion.
*/

void nsleep(long us)
{
    struct timespec wait;
    //printf("Will sleep for is %ld\n", diff); //This will take extra ~70 microseconds

    wait.tv_sec = us / (1000 * 1000);
    wait.tv_nsec = (us % (1000 * 1000)) * 1000;
    nanosleep(&wait, NULL);
}

/* ---- main entry point ---- */

int main(int argc, char **argv)
{
	//WINDOW* window;			// pointer to a window data structure
	initscr();				// fire up curses
	cbreak();				// get user input after every character
	refresh();				// prepare the console for writing
	window = newwin(24, 80, 0, 0);	// create a window with 10 rows and 40 columns at the y = 0, x = 0 on the screen

	mraa_init();
	mvwprintw(window, 0, 10,"mraa version: %s \n", mraa_get_version() );

	initGpio();	// init gpio's used as ls7366 chipselects (dir= MRAA_GPIO_OUT mode= MRAA_GPIO_STRONG, level = set_high 1)
	mvwprintw(window, 1, 10,"uart(0): %s \n", mraa_uart_get_dev_path(uart) );

	init7366(); 	// init ls7366's to do 4x decode of motor encoder quadrature inputs
		        // setup Zaxis decoder as 32bit & enable counting of encoder signals
/*
 *  loop read 32bit, printf count, delay 100ms, repeat
 */
	long microseconds = 100000;
	int a, x_axis, y_axis, z_axis;

    // figure out how long does it take to read 32bit motor encoder using galileo gen1 spi i/f
    struct timeval t1, t2;	// t1, t2 for gettimeofday()
    long long dtx, dty, dtz;	// delta_t = t2-t1 for timing getEncoderCount(Axis) function

	for(a=0; a<3600; a++){  // 3600 / (1/.1) = 3600/10 = 360 sec = 6min

		gettimeofday(&t1, NULL);
		x_axis = getEncoderCount(Xcs);
		gettimeofday(&t2, NULL);
		dtx = ((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec);

		gettimeofday(&t1, NULL);
		y_axis = getEncoderCount(Ycs);
		gettimeofday(&t2, NULL);
		dty = ((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec);

		gettimeofday(&t1, NULL);
		z_axis = getEncoderCount(Zcs);
		gettimeofday(&t2, NULL);
		dtz = ((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec);

		move(20,0);
		clrtoeol();
		move(21,0);
		clrtoeol();
		move(22,0);
		clrtoeol();
		wrefresh(window);

		mvwprintw(window, 20, 10, "X axis count\t%8X", x_axis);
		mvwprintw(window, 20, 34, "%lld", dtx);
		mvwprintw(window, 21, 10, "Y axis count\t%8X", y_axis);
		mvwprintw(window, 21, 34, "%lld", dty);
		mvwprintw(window, 22, 10, "Z axis count\t%8X", z_axis);
		mvwprintw(window, 22, 34, "%lld", dtz);
		wrefresh(window);

		nsleep(microseconds);  // delay .1sec
	}

    mraa_deinit();

    endwin();			// clear up curses resources

    return 0;
}

