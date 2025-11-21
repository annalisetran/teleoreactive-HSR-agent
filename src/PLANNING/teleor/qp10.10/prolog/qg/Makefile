#
# Makefile for "qg"
#

PROGRAM		= /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin/qg

SOURCES		= qg.ql

BASES		= $(basename $(SOURCES))
OBJECTS		= $(addsuffix .qo,$(BASES))
ASMS =        $(addsuffix .qs,$(BASES))

.PHONY:	all
all: $(PROGRAM)

$(PROGRAM): $(OBJECTS)
	$(COMPILE.qc) -o $(PROGRAM) $(OBJECTS)

.PHONY:	clean
clean:
	$(RM) Makefile 



%.qo: %.qs
	$(BOOTSTRAP.qc) -c -o $@ $<

