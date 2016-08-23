# Makefile template for shared library

# gcc -c -Wall -Werror -fPIC myTestLib.c
# gcc -shared -lrt -o libmydll.so myTestLib.o

#             ^^^^___ libmydll.so depends on librt

CC = gcc                     # C compiler
CFLAGS = -fPIC -Wall -Werror 
LDFLAGS = -shared -lrt       # link with dependent libs
RM = rm -f                   # rm command
TARGET_LIB = libmydll.so     # target lib

#SRCS = main.c src1.c src2.c # source files
SRCS = myTestLib.c
OBJS = $(SRCS:.c=.o)

.PHONY: all
all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} -o $@ $^

$(SRCS:.c=.d):%.d:%.c
	$(CC) $(CFLAGS) -MM $< >$@

include $(SRCS:.c=.d)

.PHONY: clean
clean:
	-${RM} ${TARGET_LIB} ${OBJS} $(SRCS:.c=.d)

