CC = g++

# Requirements: cpputest.github.io

BTSTACK_ROOT =  ../..

CFLAGS  = -DUNIT_TEST -g
CPPFLAGS =  -x c++ -Wall -Wno-unused
CFLAGS += -I. -I.. -I${BTSTACK_ROOT}/src 
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/micro-ecc
CFLAGS += -I${BTSTACK_ROOT}/3rd-party/rijndael
CFLAGS += -I${BTSTACK_ROOT}/platform/posix

LDFLAGS +=  -lCppUTest -lCppUTestExt

VPATH += ${BTSTACK_ROOT}/src
VPATH += ${BTSTACK_ROOT}/src/ble
VPATH += ${BTSTACK_ROOT}/platform/posix
VPATH += ${BTSTACK_ROOT}/3rd-party/micro-ecc
VPATH += ${BTSTACK_ROOT}/3rd-party/rijndael

COMMON = \
    btstack_crypto.c        	\
    btstack_linked_list.c		\
    btstack_memory.c			\
    btstack_memory_pool.c		\
    btstack_run_loop.c			\
    btstack_run_loop_posix.c    \
    btstack_util.c			    \
    hci_cmd.c					\
    hci_dump.c					\
    mock.c 				        \
    rijndael.c 					\
	
COMMON_OBJ = $(COMMON:.c=.o)

MBEDTLS = \
	ecp.c \
	ecp_curves.c \
	bignum.c \

MICROECC = \
	uECC.c

all: crypto 

crypto: ${CORE_OBJ} ${COMMON_OBJ} crypto.c
	${CC} ${CORE_OBJ} ${COMMON_OBJ} crypto.c ${CFLAGS} ${CPPFLAGS} ${LDFLAGS} -o $@

test: all
	./crypto

clean:
	rm -f  aes_ccm
	rm -f  *.o
	rm -rf *.dSYM
	
