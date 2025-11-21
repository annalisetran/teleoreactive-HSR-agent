#
# Makefile for "qp"
#
PROGRAM		= /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin/qp

SOURCES		= qp.ql

BASES		= $(basename $(SOURCES))
OBJECTS		= $(addsuffix .qo,$(BASES))
ASMS =        $(addsuffix .qs,$(BASES))

.PHONY:	all
all: $(PROGRAM)

$(PROGRAM): $(OBJECTS)
	$(COMPILE.qc) -r -o $(PROGRAM) $(OBJECTS)

.PHONY:	clean
clean:
	$(RM) Makefile $(OBJECTS) 


%.qo: %.qs
	$(COMPILE.qc) -c -o $@ $<

