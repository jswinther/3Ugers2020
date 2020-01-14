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

squareAK_3_2: squareAK_3_2.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_2.o serverif.o ${LIBS}

squareAK_3_1: squareAK_3_1.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_1.o serverif.o ${LIBS}

squareAK_3_3: squareAK_3_3.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_3.o serverif.o ${LIBS}

squareAK_3_4: squareAK_3_4.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_4.o serverif.o ${LIBS}

squareAK_3_5: squareAK_3_5.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_5.o serverif.o ${LIBS}

squareAK_3_6: squareAK_3_6.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareAK_3_6.o serverif.o ${LIBS}
clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
