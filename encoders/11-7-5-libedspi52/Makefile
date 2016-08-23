# Makefile template for shared library

# gcc -c -Wall -Werror -fPIC lib_edspi52.c
# gcc -shared -lrt -lmraa -o libedspi52.so lib_edspi52.o

#             ^^^^__^^^^^  depends on librt & libmraa

CC = gcc # C compiler
CFLAGS = -fPIC -Wall -Werror
LDFLAGS = -shared -lrt -lmraa # link with dependent libs
RM = rm -f # rm command
TARGET_LIB = libedspi52.so # target lib

#SRCS = main.c src1.c src2.c # source files
SRCS = lib_edspi52.c
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

