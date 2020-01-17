#
# The Compiler
#
CC      = gcc
LD      = ${CC}
SMR     = /usr/local/smr
CFLAGS  = -Wall -O2 -I${SMR}/include 
LDFLAGS = -L${SMR}/lib 

#
# Our program files
#
PROG   = square
HDRS   =
OBJS   = square.o serverif.o
LIBS   = -lm /usr/local/smr/lib/librhd.a -lrobot

all:	${PROG}

${PROG}: ${OBJS}
	${LD} -o ${@} ${LDFLAGS} ${OBJS} ${LIBS}

main: main.o serverif.o .FORCE
	${LD} -o ${@} ${LDFLAGS} main.o serverif.o ${LIBS} && ./main

test: test.o serverif.o .FORCE
	${LD} -o ${@} ${LDFLAGS} test.o serverif.o ${LIBS} && ./test

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
