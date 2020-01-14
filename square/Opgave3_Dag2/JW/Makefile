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

square_modified: square_modified.o serverif.o
	${LD} -o ${@} ${LDFLAGS} square_modified.o serverif.o ${LIBS}

squareTP: squareTP.o serverif.o
	${LD} -o ${@} ${LDFLAGS} squareTP.o serverif.o ${LIBS}

jw_ex31: jw_ex31.o serverif.o
	${LD} -o ${@} ${LDFLAGS} jw_ex31.o serverif.o ${LIBS}

clean:
	rm -f ${OBJS}

${OBJS}: ${HDRS} Makefile
