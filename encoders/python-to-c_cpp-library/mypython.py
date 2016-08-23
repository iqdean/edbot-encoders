import ctypes
import time
import math

''' 
cdll loads libraries which export functions using the standard cdecl calling convention 
'''

'''
For WINDOWS Only: 
lib_astro = ctypes.cdll.mydll '''

lib_astro = ctypes.cdll.LoadLibrary("./libmydll.so")

'''
define geoTime class which represents the c struct used to pass data
back and forth b/t python code and c/c++ library

typedef struct {
  uint32_t seconds;
  uint32_t nanoseconds;
} geoTime;

'''

class geoTime(ctypes.Structure):
    _fields_ = [("seconds", ctypes.c_uint),
                ("nanoseconds", ctypes.c_uint)]  

'''
time.time() 
Return the time as a floating point number expressed in seconds since the epoch, in UTC
'''
now = time.time()
print "Python Now: ", now

'''
instance now_geoTime of type geoTime
& set geoTime to now.                      now is floating point          Int.Frac
geoTime.seconds <- math.floor(now)         math.floor(now) returns        Int 
geoTime.nanoseconds <- math.modf(now)[0]   math.modf(now)[0] returns      Frac
                            *10^9          converts Frac to nanoseconds
'''

now_geoTime = geoTime()
now_geoTime.seconds = ctypes.c_uint(int((math.floor(now))))
now_geoTime.nanoseconds = ctypes.c_uint(int(math.floor(math.modf(now)[0] * 1000000000)))
print "Python geoTime now:", now_geoTime.seconds, now_geoTime.nanoseconds

'''
setup argument types for function myTest() in mydll
int myTest(geoTime *myTime)
 |            \______________ [ctypes.POINTER(geoTime)]
 |         \\\\\\\\\\\\\\\\__ ctypes.byref(now_geoTime)  << cuz input param is a *ptr
 \___________________________  ctypes.c_uint             RETURN TYPE IS int

'''

lib_astro.myTest.argtypes = [ctypes.POINTER(geoTime)]
lib_astro.myTest.restype = ctypes.c_uint

'''
NOTE: TODO 
verify we can return a pointer to a struct from the shared library
and read it's contents... will require memcpy

lib_astro.myTest2.argtypes = [ctypes.POINTER(geoTime)]
lib_astro.myTest2.restype = [ctypes.POINTER(geoTime)]

geoTime * myTest2(geoTime *myTime)

'''

print "************* ENTERING C ********************"

'''
 CALL func int myTest(geoTime *myTime) in mydll
'''

test = lib_astro.myTest(ctypes.byref(now_geoTime))

print "************* EXITING C **********************"

print "Modified now_geoTime: ",now_geoTime.seconds, now_geoTime.nanoseconds
print "test: ",test

print "\nDelay .5 sec\n"
time.sleep(.5)

now = time.time()
print "Python Now: ", now
now_geoTime.seconds = ctypes.c_uint(int((math.floor(now))))
now_geoTime.nanoseconds = ctypes.c_uint(int(math.floor(math.modf(now)[0] * 1000000000)))
print "Python geoTime now:", now_geoTime.seconds, now_geoTime.nanoseconds

'''
setup argument types for function patTest() in mydll
geoTime patTest(geoTime *myTime)
 |               \______________ [ctypes.POINTER(geoTime)]
 |               \\\\\\\\\\\\__ ctypes.byref(now_geoTime)  << cuz input param is a *ptr
 \_____________________________ geoTime                    RETURN TYPE = geoTime

'''

lib_astro.patTest.argtypes = [ctypes.POINTER(geoTime)]
lib_astro.patTest.restype = geoTime
print "************* ENTERING C ********************"
t1 = time.time()
test = lib_astro.patTest(ctypes.byref(now_geoTime))
t2 = time.time()
print "************* EXITING C **********************"
print "Modified now_geoTime: ",now_geoTime.seconds, now_geoTime.nanoseconds
print "Type of test: ",test
print "Information in test: ", test.seconds, test.nanoseconds

t1_geoTime = geoTime()
t2_geoTime = geoTime()

print "t2    : ", t2
t2_geoTime.nanoseconds = ctypes.c_uint(int(math.floor(math.modf(t2)[0] * 1000000000)))
print "t2.ns : ", t2_geoTime.nanoseconds

print "t1    : ", t1
t1_geoTime.nanoseconds = ctypes.c_uint(int(math.floor(math.modf(t1)[0] * 1000000000)))
print "t1.ns : ", t1_geoTime.nanoseconds

print "t2-t1 ns : ", t2_geoTime.nanoseconds - t1_geoTime.nanoseconds

'''
linux-best-ctypes-ref-yet$ python mypython.py 
Python Now:  1450769384.39
Python geoTime now: 1450769384 390630960
************* ENTERING C ********************
Time: 1450769384 390630960
MyTime: 390630960 390630960      retValue: 314 159
************* EXITING C **********************
Modified now_geoTime:  390630960 390630960
test:  314

Delay .5 sec

Python Now:  1450769384.89
Python geoTime now: 1450769384 890993118
************* ENTERING C ********************
Time: 1450769384 890993118
MyTime: 890993118 890993118      retValue: 1450769384 891175471
************* EXITING C **********************
Modified now_geoTime:  890993118 890993118
Type of test:  <__main__.geoTime object at 0x7f8d14e8d7a0>
Information in test:  1450769384 891175471
t2    :  1450769384.89
t2.ns :  891189098
t1    :  1450769384.89
t1.ns :  891160011
t2-t1 ns :  29087 = 29.087uS
'''
