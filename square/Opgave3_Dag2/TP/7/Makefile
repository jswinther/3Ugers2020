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

squareAK: squareAK.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK.o serverif.o ${LIBS}

squareTPline: squareTPline.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareTPline.o serverif.o ${LIBS}

squareTPdir: squareTPdir.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareTPdir.o serverif.o ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
