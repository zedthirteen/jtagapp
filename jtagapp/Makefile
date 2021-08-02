#################################################################################
#
# 2021-07-29 DJF
#
# Main jtag application
#
# Expected folder structure is:
#
#	~/jtagdev/jtagapp	# this folder - where application is built and executed
#	~/jtagdev/jtaginc	# all include files common to libjtag_funcs.a and jtag application
#	~/jtagdev/jtaglib	# source files for libjtag_funcs.a - not distributed
#       ~/jtag			# target directory for application build and execution
#
# Note: this folder must have libjtag_funcs.a before building
#
# use "make clean jtag" to build
#
#################################################################################

#
# DJF Note: this file was derived from wiringPi project
#

#
# Makefile:
#	wiringPi - Wiring Compatable library for the Raspberry Pi
#	https://projects.drogon.net/wiring-pi
#
#	Copyright (c) 2012-2015 Gordon Henderson
#################################################################################
# This file is part of wiringPi:
#	Wiring Compatable library for the Raspberry Pi
#
#    wiringPi is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    wiringPi is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
#################################################################################


ifneq ($V,1)
Q ?= @
endif

DEBUG	= -O3
CC	= gcc
INCLUDE	= -I/usr/local/include \
          -I../jtaginc
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LDFLAGS	= -L/usr/local/lib \
		-L.
LDLIBS	= -lwiringPi -lwiringPiDev -lpthread -ljtag_funcs -lm

SRC	= 	jtag.c

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:
	$Q cat README.md
	$Q echo "    $(BINS)" | fmt
	$Q echo ""

really-all:	$(BINS)

jtag:	jtag.o
	$Q echo [link]
	$Q $(CC) -o $@ jtag.o $(LDFLAGS) $(LDLIBS)
	cp jtag ../../jtag/jtag
	cp -u jtag.ini ../../jtag/jtag.ini

.c.o:
	$Q echo [CC] $<
	$Q $(CC) -c $(CFLAGS) $< -o $@

clean:
	$Q echo "[Clean]"
	$Q rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	$Q echo [ctags]
	$Q ctags $(SRC)

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
