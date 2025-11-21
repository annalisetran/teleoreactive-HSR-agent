#
# Main makefile
#
# Please note that the order in which the targets are evaluated
# is important. Don't be tempted to reorder or parallelise
# the makefiles!
#
# Richard Hagen
#

#
# Nothing else should need attention, but there are some things that
# are interesting later on, particularly the rules for making Qu-Prolog
# objects.
#

export QPBIN	= /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin


export QC		= $(QPBIN)/qc
export QC_RELEASE	= $(QC)
export QCFLAGS		= 
export COMPILE.qc	= $(QC) $(QCFLAGS)
export BOOTSTRAP.qc	= $(QC) $(QCFLAGS)

export PROLOG		= prolog



.PHONY: all
all:	objects

#export	PARALLEL	= -j 2

.PHONY: objects
objects:
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin	$(PARALLEL) all 
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src	$(PARALLEL) all
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src	$(PARALLEL) install
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/prolog	$(PARALLEL) all




.PHONY: xqp
xqp:
	cd /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqp;qmake;cd -
	cd /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqpdebug;qmake;cd -
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqp
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqpdebug
	/usr/bin/install /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqp/xqp /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin
	/usr/bin/install /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src/xqpdebug/xqpdebug /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/bin

.PHONY: clean
clean:
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/src	$(PARALLEL) clean
	@$(MAKE) -C /home/blinky/roboworkspace/src/PLANNING/teleor/qp10.10/prolog	$(PARALLEL) clean
