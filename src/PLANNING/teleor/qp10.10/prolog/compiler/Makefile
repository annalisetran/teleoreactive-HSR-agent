#
# Makefile for "qu-prolog/compiler"
#
QPOBJDIR =/home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/prolog_object/compiler

SOURCES	= \
	compile.ql \
	compile_pred.ql \
	custom.ql \
	flatten.ql \
	index.ql \
	internal_form.ql \
        lastxreg.ql \
	list_util.ql \
	pretrans.ql \
	pseudoinstr.ql \
	special.ql \
	write.ql \
	xregs.ql

BASES	= $(basename $(SOURCES))
OBJECTS = $(addsuffix .qo,$(BASES))
ASMS =        $(addsuffix .qs,$(BASES))



.PHONY:	all
all: $(OBJECTS)

.PHONY:	clean
clean:
	$(RM)  Makefile


%.qo: %.qs
	$(BOOTSTRAP.qc) -c -o $@ $<

