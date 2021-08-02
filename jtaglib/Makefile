#################################################################
#
# 2021-07-29 DJF
#
# Makefile to build libjtag_funcs.a to use in main application
#
# expected folder structure is:
#
#	~/jtaglib		# this folder
#	~/jtaginc		# all header files
#	~/jtag			# not used for lib creation
#
# use "make clean all" in this folder
#
#################################################################


DESTDIR	= ../jtagapp

STATIC	= $(DESTDIR)/libjtag_funcs.a

CC	= gcc
INCLUDE	= -I../jtaginc -I/usr/local/include
CFLAGS	= -Wall $(INCLUDE) -Winline -pipe

LDFLAGS	= -static
LDLIBS	= -lwiringPi -lwiringPiDev -lpthread -lm

SRC	= jtag_funcs.c ini.c

OBJ	= $(SRC:.c=.o)

$(STATIC):	$(OBJ)
	@echo "[Link (Static)]"
	@ar rcs $@ $^

.c.o:
	@echo [Compile] $<
	@$(CC) -c $(CFLAGS) $< -o $@

.PHONY: all
all: $(STATIC)

.PHONY: clean
clean:
	$Q echo "[Clean]"
	rm -f $(STATIC) $(OBJ) *~ core tags


# DO NOT DELETE


