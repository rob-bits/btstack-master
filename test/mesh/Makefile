# Makefile for libusb based examples
BTSTACK_ROOT = ../..

CORE += \
	btstack_link_key_db_fs.c \
	btstack_run_loop_posix.c \
	btstack_uart_block_posix_pty.c \
	hci_transport_h4.c \
	le_device_db_fs.c \
	main.c \
	btstack_stdin_posix.c \
	wav_util.c 					\

# examples
include ${BTSTACK_ROOT}/example/Makefile.inc

CFLAGS  += -g -Wall -Werror \
	-I$(BTSTACK_ROOT)/platform/embedded \
	-I$(BTSTACK_ROOT)/platform/posix \

VPATH += ${BTSTACK_ROOT}/platform/posix
VPATH += ${BTSTACK_ROOT}/platform/embedded

ifeq ($(OS),Windows_NT)
LDFLAGS += -lws2_32
endif

# Command Line examples require porting to win32, so only build on other unix-ish hosts
ifneq ($(OS),Windows_NT)
EXAMPLES += ${EXAMPLES_CLI}
endif

EXAMPLES=le_counter

all: ${EXAMPLES}

