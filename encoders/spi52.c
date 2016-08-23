/*  <ike.a.dean> iad1046@gmail.com
 *  spi52.c - Read 32bit quadrature encoder counter from spi-based LS7366 shield
 *
 *  To Build on iot-devkit linux image:
 *  $ gcc -Wall -o spi52 spi52.c -lmraa -lncurses
 *
 *  Revision History
 *  12-08-14 - 1st contact: successfully reads Z-axis encoder
 *  12-10-14 - display XYZ axis sensor data with ncurses (w/o scrolling it w printf's)
 *  12-12-14 - time spi bus xfers used by getEncoder() using gettimeofday
 *             on x86, gettimeofday() provides uSec resolution on modern linux kernels
 *  NEXT:
 *  1. figure out how to setup git repo for Galileo based Differentional Drive Robot Base Controller for use w ROS (gen1_ros_rbc)
 *  2. enable Linux Kernel real time subsystem to run motor control loop
 *     2.1 customize iotdevkit linux kernel config to enable CONFIG_PREMEPT_RT
 *     2.2 adapt Linux Realtime cycletest.c app to toggle non-ioexpander gpio
 *     2.3 scope the non-ioexpander gpio while loading the linux kernel to confirm realtime code is working
 *  3. update cycletest.c code to turn it into motor control loop
 */

/*
 * 12-14-14  spi2.c output
          mraa version: v0.5.2-18-g9014fc6
          spi = mraa_spi_init(0): Ok

          Set cs[X Y Z] = 1

          Xcs WR_MDR0: 88
          Xcs RD_MDR0: 3
          Xcs WR_MDR1: 90
          Xcs RD_MDR0: 0

          Ycs WR_MDR0: 88
          Ycs RD_MDR0: 3
          Ycs WR_MDR1: 90
          Ycs RD_MDR0: 0

          Zcs WR_MDR0: 88
          Zcs RD_MDR0: 3
          Zcs WR_MDR1: 90
          Zcs RD_MDR0: 0

          X axis count  0               6191 us
          Y axis count  FFFFFFF7        5979 us
          Z axis count  7               5901 us

Analysis/Issues:
1. takes ~6.2ms to read a 32byte counter using galileo gen1 spi i/f ... THAT IS TERRIBLE!!
   6191us = 6.191ms  6.2ms per encoder  12.4ms to read 2 encoders
   50hz motor control loop has to run every 1/50 20ms, if it takes 12.4ms just to read the encoders is over 1/2 of 20ms budget!!
2. for static encoder count, the spi reads are not stable, the most significant byte exibits noise every once in a while ??
   ex:   X axis count   0 -> F8000000 -> 0  \
         Z axis count   7 -> 80000007 -> 7   > not acceptable for motor control apps... will have to debug or filter
                             F0000007       /  to mke this usable

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
int csX = 5;
int csY = 9;
int csZ = 6;

mraa_spi_context spi;
mraa_gpio_context Xcs;
mraa_gpio_context Ycs;
mraa_gpio_context Zcs;

int set_high = 1;
int set_low = 0;
uint8_t txb;
uint8_t rxb;

WINDOW* window;			// pointer to ncurses window

/* ------- function defs ------*/

/*
This routine is junk... it was copied from libmraa git...and when used as is this app becomes UNSTABLE!
The program will correctly read LS7366 quadrature counter for about a minute
& then all of the sudden start reading garbage... not sure what exactly was causing the instability:
? repeatly calling mraa_gpio_init() everytime the chip select signal needs to be transitioned
? not setting CY8C940 drive mode using mraa_gpio_mode()
After much twiddling and restructuring of the code to what it is now, the app appears to be much more stable
but still seeing anomolies in the expected counter readings every once in a while:
0 becomes F0000000   or   1024 becomes E0001024		< but then on the very next read, .5sec later the correct value shows up
> still can't rely on the readings for motor pid loop application w/o further filtering to remove the anomolies
Next:
> look at LS7366 datasheet to make sure we correctly capturing the 32bit counter and aren't catching catching it just as it changes
> Slow down the spi clock and see if that helps, right now it should be running at default 4mhz rate
  figure out how to print time delay from nsleep so we can time the spi transactions and see how long it takes to read encoder
> figure out how to use ncurses

mraa_result_t gpio_set(int pin, int level) {
    mraa_gpio_context gpio = mraa_gpio_init(pin);	// there should be a mraa_gpio_close(gpio) ?
    if (gpio != NULL) {
        mraa_gpio_dir(gpio, MRAA_GPIO_OUT);         // no mraa_gpio_mode(gpio, [strong,pullup/dwn,etc] ) ?
        mraa_gpio_write(gpio, level);
        return MRAA_SUCCESS;
    } else
        return MRAA_ERROR_INVALID_RESOURCE;
}

*/

//                         cspin = Xcs:Ycs:Zcs
mraa_result_t gpio_set(mraa_gpio_context csPin, int level) {
	return mraa_gpio_write(csPin, level);
}

/*
 * init gpio pins to be used for chip selects
 */
void initGpio() {

	Xcs = mraa_gpio_init(csX);
	Ycs = mraa_gpio_init(csY);
	Zcs = mraa_gpio_init(csZ);

	mraa_gpio_dir(Xcs, MRAA_GPIO_OUT);  // set directioon
	mraa_gpio_dir(Ycs, MRAA_GPIO_OUT);
	mraa_gpio_dir(Zcs, MRAA_GPIO_OUT);

	mraa_gpio_mode(Xcs, MRAA_GPIO_STRONG); // set output pin cfg
	mraa_gpio_mode(Ycs, MRAA_GPIO_STRONG); // STRONG ?or? MRAA_GPIO_PULLUP
	mraa_gpio_mode(Zcs, MRAA_GPIO_STRONG);

	gpio_set(Xcs, set_high);
	gpio_set(Ycs, set_high);
	gpio_set(Zcs, set_high);

	return;
}

int rd7366SpiReg(mraa_gpio_context csPin, int cmd){		// 2 byte operation
	gpio_set(csPin, set_low);	//rw transaction initiated on falling edge of chip select
	txb = cmd;
	rxb = mraa_spi_write(spi,txb);	//1st txb = reg_r_cmd rxb = junk
	rxb = mraa_spi_write(spi,txb);  //2nd txb = junk      rxb = r_reg_value
	gpio_set(csPin, set_high);
	return rxb;
}

void wr7366SpiReg(mraa_gpio_context csPin, int cmd, int val){ 	// 2 byte operation
	gpio_set(csPin, set_low);
	txb = cmd;
	rxb = mraa_spi_write(spi,txb);	//1st txb = reg_w_cmd rxb = junk
	txb = val;
	rxb = mraa_spi_write(spi,txb);	//2nd txb = w_reg_val rxb = junk
	gpio_set(csPin, set_high);
	return;
}

void init7366(){
    gpio_set(Xcs, set_high);
    gpio_set(Ycs, set_high);
    gpio_set(Zcs, set_high);				// set all chip selects = 1
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
	rxb = mraa_spi_write(spi,txb);	// 1 txb = RD_CNTR  rxb = junk
	bv3 = mraa_spi_write(spi,txb);	// 2 txb = junk     rxb = bv3	msb
	bv2 = mraa_spi_write(spi,txb);	// 3 txb = junk     rxb = bv2
	bv1 = mraa_spi_write(spi,txb);	// 4 txb = junk     rxb = bv1
	bv0 = mraa_spi_write(spi,txb);	// 5 txb = junk     rxb = bv0	lsb

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

	spi = mraa_spi_init(0);
	mvwprintw(window, 1, 10,"spi = mraa_spi_init(0): Ok\n");

	initGpio();	// init gpio's used as ls7366 chipselects (dir= MRAA_GPIO_OUT mode= MRAA_GPIO_STRONG, level = set_high 1)
	init7366(); 	// init ls7366's to do 4x decode of motor encoder quadrature inputs
		        // setup Zaxis decoder as 32bit & enable counting of encoder signals


/*
 *  loop read 32bit, printf count, delay 100ms, repeat
 */
	long microseconds = 500000;
	int a, x_axis, y_axis, z_axis;
    // figure out how long does it take to read 32bit motor encoder using galileo gen1 spi i/f
    struct timeval t1, t2;	// t1, t2 for gettimeofday()
    long long dtx, dty, dtz;	// delta_t = t2-t1 for timing getEncoderCount(Axis) function

	for(a=0; a<3600; a++){  // 3600 x .5sec = 30min

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

		mvwprintw(window, 20, 10, "X axis count\t%8X", x_axis);
		mvwprintw(window, 20, 34, "%lld", dtx);
		mvwprintw(window, 21, 10, "Y axis count\t%8X", y_axis);
		mvwprintw(window, 21, 34, "%lld", dty);
		mvwprintw(window, 22, 10, "Z axis count\t%8X", z_axis);
		mvwprintw(window, 22, 34, "%lld", dtz);
		wrefresh(window);

		nsleep(microseconds);  // delay .5sec
	}

    mraa_deinit();

    endwin();			// clear up curses resources

    return 0;
}

