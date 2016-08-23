import ctypes
import time
import math

'''
typedef struct {
  uint32_t x_enc_cnt;     // x-axis encoder count
  uint32_t x_ts_sec;      // x-axis timestamp (sec:nsec)
  uint32_t x_ts_ns;
  uint32_t y_enc_cnt;     // y-axis encoder count
  uint32_t y_ts_sec;      // y-axis timestamp (sec:nsec)
  uint32_t y_ts_ns;
} mtrEnc;
'''

class mtrEnc(ctypes.Structure):
    _fields_ = [("x_enc_cnt", ctypes.c_uint),
                ("x_ts_sec", ctypes.c_uint),
                ("x_ts_ns", ctypes.c_uint),
                ("y_enc_cnt", ctypes.c_uint),
                ("y_ts_sec", ctypes.c_uint),
                ("y_ts_ns", ctypes.c_uint)]

'''
REF: 
http://interactivepython.org/runestone/static/pythonds/BasicDS/ImplementingaQueueinPython.html
assumes rear is at position 0 in the list.
insert - function adds new elements to the rear of the queue. (the 1st element of the list)
pop    - operation used to remove the front element (the last element of the list)
'''

class Queue:
    def __init__(self):
        self.items = []

    def isEmpty(self):
        return self.items == []

    def enqueue(self, item):
        self.items.insert(0,item)

    def dequeue(self):
        return self.items.pop()

    def size(self):
        return len(self.items)

mylib = ctypes.cdll.LoadLibrary("./libedspi52.so")

mylib.edspi52_init.argtypes = None
mylib.edspi52_init.restype = ctypes.c_uint

mylib.edspi52_deinit.argtypes = None
mylib.edspi52_deinit.restype = ctypes.c_uint

mylib.getXYEncCount.argtypes = None
mylib.getXYEncCount.restype = mtrEnc

print "****** call int edspi52_init(void) *********"
test = mylib.edspi52_init(None)
print "ret val : ", test

q = Queue()

print "Calling mtrEnc getXYEncCount(void) 10 times ..."

for num in range(0,10):
	''' print "Calling mtrEnc getXYEncCount(void): ", num '''
        ''' getXYEncCount() returns mtrEnc struct on the stack'''
	test = mylib.getXYEncCount(None)
        q.enqueue(test)
	''' print "x_enc: ", test.x_enc_cnt, test.x_ts_sec, test.x_ts_ns '''
	''' print "y_enc: ", test.y_enc_cnt, test.y_ts_sec, test.y_ts_ns '''
	''' time.sleep(0.5)'''

qmax = q.size()

print "Dumping queue ... qmax : ", qmax

for i in range(q.size()):
    test = q.dequeue()
    print "q.size() - i : ", (qmax - i)
    print "q.dequeue x_enc: %8X %d %d" % (test.x_enc_cnt, test.x_ts_sec, test.x_ts_ns)
    print "q.dequeue y_enc: %8X %d %d" % (test.y_enc_cnt, test.y_ts_sec, test.y_ts_ns)

print "****** call int edspi52_deinit(void) *****"
test = mylib.edspi52_deinit(None)
