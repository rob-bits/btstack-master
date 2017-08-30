# Makefile for libusb based examples
BTSTACK_ROOT = ../..

CORE += \
	adv_bearer.c \
	beacon.c \
	btstack_link_key_db_fs.c \
	btstack_run_loop_posix.c \
	btstack_stdin_posix.c \
	btstack_uart_block_posix_pty.c \
	hci_transport_h2_libusb.c \
	hci_transport_h4.c \
	le_device_db_fs.c \
	main.c \

# examples
include ${BTSTACK_ROOT}/example/Makefile.inc

CFLAGS  += -g -Wall -Werror \
	-I$(BTSTACK_ROOT)/platform/embedded \
	-I$(BTSTACK_ROOT)/platform/posix \

VPATH += ${BTSTACK_ROOT}/src/ble/mesh
VPATH += ${BTSTACK_ROOT}/platform/posix
VPATH += ${BTSTACK_ROOT}/platform/embedded
VPATH += ${BTSTACK_ROOT}/platform/libusb

# libusb
CFLAGS  += $(shell pkg-config libusb-1.0 --cflags)
LDFLAGS += $(shell pkg-config libusb-1.0 --libs)

mesh: ${CORE_OBJ} ${COMMON_OBJ} ${SM_OBJ} mesh.c 
	${CC} $^ ${CFLAGS} ${LDFLAGS} -o $@

EXAMPLES= mesh

all: ${EXAMPLES}

