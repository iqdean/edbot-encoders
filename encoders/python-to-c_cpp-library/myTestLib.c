/****************************************************************
http://stackoverflow.com/questions/10484995/python-ctypes-passing-in-pointer-and-getting-struct-back
 
To build on windows with MinGW:

PATH=C:\MinGW\bin;%PATH%                    OUTPUTS
gcc -c myTestLib.c                          myTestLib.o
gcc -shared -o mydll.dll myTestLib.o        mydll.dll

To see functions exported by mydll.dll, open mydll.dll with
dependency walker tool (depends22)

To build on Linux with gcc:
                                            OUTPUTS
gcc -c -Wall -Werror -fpic myTestLib.c      myTestLib.o
gcc -shared  -o libmydll.so myTestLib.o     libmydll.so

To see functions exported by libmydll.so, use nm:
$ nm -C -D -g libmydll.so 

To set breakpoint in shared lib called from python:

1. bld the lib w debug symbols:

gcc -c -Wall -Werror -fpic myTestLib.c -g
gcc -shared  -o libmydll.so myTestLib.o

2. $ sudo apt-get install gdb python2.7-dbg

3. REF: https://wiki.python.org/moin/DebuggingWithGdb

$ gdb python

(gdb) b myTest
Function "myTest" not defined.
Make breakpoint pending on future shared library load? (y or [n]) y

Breakpoint 1 (myTest) pending.

(gdb) run mypython.py
Starting program: /usr/bin/python mypython.py
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
Python Now:  1450631588.87
Python geoTime now: 1450631588 865993976
************* ENTERING C ********************

Breakpoint 1, myTest (myTime=0x7ffff7ec66d0) at myTestLib.c:83
warning: Source file is more recent than executable.
83	
(gdb) x/2xw 0x7ffff7ec66d0
0x7ffff7ec66d0:	0x5676e1a4	0x339e04f8
(gdb) 
                1450631588       865993976    < matches now_geoTime OK

Hmm, seems ctypes.byref(now_geoTime) is working and the problem is
de-referencing the pointer in the printf statement

PROBLEM WAS due to:
                                   size on 32bit    size on 64bit                 
typedef unsigned long uint32_t;         32               64

FIX: 

typedef unsigned int uint32_t;
& 
Chg all the printf %lu -> %d

NOW IT WORKS ON 64bit ubu14.04 Linux:

$ python mypython.py
Python Now:  1450633688.23
Python geoTime now: 1450633688 234457015
************* ENTERING C ********************
Time: 1450633688 234457015
MyTime: 234457015 234457015      retValue: 314 159
************* EXITING C **********************
Modified now_geoTime:  234457015 234457015
test:  314

Delay .5 sec

Python Now:  1450633688.74
Python geoTime now: 1450633688 735325098
************* ENTERING C ********************
Time: 1450633688 735325098
MyTime: 735325098 735325098      retValue: 123 567
************* EXITING C **********************
Modified now_geoTime:  735325098 735325098
Type of test:  <__main__.geoTime object at 0x7f1b7113b7a0>
Information in test:  123 567

12-21-2015  - update patTest() func to 
 a) read the current sec.nsec into struct timespec t1 using clock_gettime() api of librt 
    REF:  http://blog.stalkr.net/2010/03/nanosecond-time-measurement-with.html

 b) set geoTime T  T.sec < t1.sec & T.nsec < t1.nsec

 c) return geoTime;    < returns geoTime T on the stack

To Build on Linux, 

 now libmydll.so uses another shared library called librt.so 
 librt.so - rqd for clock_gettime() api to read current time in sec.nanosec timespec format

 gcc -c -Wall -Werror -fpic myTestLib.c -lrt       -lxxx specs the library to link againest 
 gcc -shared  -o libmydll.so myTestLib.o            which in this case is librt

linux-best-ctypes-ref-yet$ python mypython.py

Python Now:  1450743485.18
Python geoTime now: 1450743485 175621032
************* ENTERING C ********************
Time: 1450743485 175621032
MyTime: 175621032 175621032      retValue: 314 159
************* EXITING C **********************
Modified now_geoTime:  175621032 175621032
test:  314

Delay .5 sec

Python Now:  1450743485.68
Python geoTime now: 1450743485 675889015
************* ENTERING C ********************
Time: 1450743485 675889015
MyTime: 675889015 675889015      retValue: 1450743485 676043236
************* EXITING C **********************
Modified now_geoTime:  675889015 675889015
Type of test:  <__main__.geoTime object at 0x7f59ce88f7a0>
Information in test:  1450743485 676043236

675889015 - 175621032 = 500422204 nsec = .500422204 sec 
                        999999999 nsec   .999999999 sec

Units seem to work out and the delta is ~.5sec which is what would
be expected
 

 
*****************************************************************/
//typedef unsigned long uint32_t;  < works on windows w 32bit mingw
//                                   size on 32bit    size on 64bit
//                                      32bits           64bits

typedef unsigned int uint32_t;

typedef struct {
  uint32_t seconds;
  uint32_t nanoseconds;
} geoTime;


/* remove this and use EXPORTIT ifdef to make code build on 
   windows and linux

int __declspec(dllexport) myTest(geoTime *myTime);
geoTime __declspec(dllexport) patTest(geoTime *myTime);

*/

#ifdef _WIN32
#  define EXPORTIT __declspec( dllexport )
#else
#  define EXPORTIT
#endif


#include <stdio.h>
#include <time.h>

EXPORTIT int myTest(geoTime *myTime){
  printf("Time: %d %d\n", myTime->seconds, myTime->nanoseconds);
  myTime->seconds = myTime->nanoseconds;
  geoTime T = {314, 159};    /* def variable T of type geoTime & init it to sec=314 ns=159 */
  printf("MyTime: %d %d      retValue: %d %d\n", myTime->seconds, myTime->nanoseconds, T.seconds, T.nanoseconds);
  return 314;
}

EXPORTIT geoTime patTest(geoTime *myTime){

    struct timespec t1;
    geoTime T;

    if (clock_gettime(CLOCK_REALTIME, &t1) != 0)
    {
    	perror("clock_gettime");
	T.seconds = -1;
	T.nanoseconds = -1;
    	return T;
    }

    printf("Time: %d %d\n", myTime->seconds, myTime->nanoseconds);
    myTime->seconds = myTime->nanoseconds;

    T.seconds = t1.tv_sec;
    T.nanoseconds = t1.tv_nsec;

    printf("MyTime: %d %d      retValue: %d %d\n", myTime->seconds, myTime->nanoseconds, T.seconds, T.nanoseconds);
    return T;     // geoTime T.seconds & T.nanoseconds is returned on the stack
}
