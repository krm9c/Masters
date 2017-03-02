# Makefile for Aim104 drivers and libraries.

ifneq ($(KERNELRELEASE),)
include $(src)/Kbuild
else

PACKAGE := aim104
VERSION := $(shell ael_changelog --upstream)
export PACKAGE VERSION

# Directories

PREFIX := /usr
LIBDIR := $(PREFIX)/lib
INCDIR := $(PREFIX)/include
BINDIR := $(PREFIX)/bin
export PREFIX LIBDIR INCDIR BINDIR

WSDDPREFIX := /opt/wsdd5.0
WSDDLIBDIR := $(WSDDPREFIX)/lib
WSDDBINDIR := $(WSDDPREFIX)/bin
export WSDDPREFIX WSDDLIBDIR WSDDBINDIR

all: lib lib-java

# Makefile rules for 2.4 modules.

# Change this directory to point to the correct kernel source for the
# kernel version that is on the target.
KSRC ?= /usr
INCLUDEDIR := $(KSRC)/include

CFLAGS = -D__KERNEL__ -DMODULE -DEXPORT_SYMTAB -O3 -Wall \
               -I$(INCLUDEDIR) -I./include

OBJS = aim104.o aim104-relay8.o aim104-in16.o aim104-out16.o aim104-io32.o \
               aim104-multi-io.o aim104-dummy.o

UTS_RELEASE := $(shell grep UTS_RELEASE $(INCLUDEDIR)/linux/version.h | cut -f2 -d\")

modules: module24 # for backward compatibility

modules24: $(OBJS)

modules24_install: modules24
	install -d $(DESTDIR)/lib/modules/$(UTS_RELEASE)/arcom
	set -e ; for O in $(OBJS) ; do \
		install -o root -g root -m 0644 $$O $(DESTDIR)/lib/modules/$(UTS_RELEASE)/arcom ; \
	done

aim104.o: aim104.c ./include/linux/arcom/aim104.h
aim104-relay8.o: aim104-relay8.c ./include/linux/arcom/aim104.h
aim104-in16.o: aim104-in16.c ./include/linux/arcom/aim104.h
aim104-out16.o: aim104-out16.c ./include/linux/arcom/aim104.h
aim104-io32.o: aim104-io32.c ./include/linux/arcom/aim104.h
aim104-multi-io.o: aim104-multi-io.c ./include/linux/arcom/aim104.h
aim104-dummy.o: aim104-dummy.c include/linux/arcom/aim104.h


modules26:
	$(MAKE) -C $(KSRC) M=$(PWD) modules

modules26_install:
	$(MAKE) -C $(KSRC) M=$(PWD) INSTALL_MOD_PATH=$(DESTDIR) modules_install

# C library

.PHONY: lib
lib:
	cd lib && $(MAKE) lib

# Java library

lib-java:
	cd java && $(MAKE) lib-java

# GCJ library

lib-gcj:
	cd java && $(MAKE) lib-gcj

# Tests

test: test-modules test-lib test-java test-jxe

# Kernel module tests

TEST_PROGS = test_aim104-relay8 test_aim104-in16 test_aim104-out16 \
		test_aim104-io32 test_aim104-multi-io

test-modules: CFLAGS = -I./include -g -Wall

test-modules: $(TEST_PROGS)

test_aim106-relay8: test_aim104-relay8.o
test_aim104-in16: test_aim104-in16.o
test_aim104-out16: test_aim104-out16.o
test_aim104_io32: test_aim104-io32.o
test_aim104-multi-io: test_aim104-multi-io.o

test_aim104-relay8.o: test_aim104-relay8.c ./include/linux/arcom/aim104.h
test_aim104-in16.o: test_aim104-in16.c ./include/linux/arcom/aim104.h
test_aim104-out16.o: test_aim104-out16.c ./include/linux/arcom/aim104.h
test_aim104-io32.o: test_aim104-io32.c ./include/linux/arcom/aim104.h
test_aim104-multi-io.o: test_aim104-multi-io.c ./include/linux/arcom/aim104.h

# C library tests

test-lib:
	cd lib && $(MAKE) test-lib

# Java library tests

test-java:
	cd java && $(MAKE) test-java

# Java library tests using JXEs

test-jxe:
	cd java && $(MAKE) test-jxe

# GCJ library tests

test-gcj:
	cd java && $(MAKE) test-gcj

# Documentation

.PHONY: docs
docs:
	cd docs && $(MAKE) docs
	cd java && $(MAKE) docs

# Install

.PHONY: install
install:
	cd lib && $(MAKE) install
	cd java && $(MAKE) install
	install -d $(DESTDIR)$(BINDIR)
	set -e ; for TEST in $(TEST_PROGS) ; do \
		install -m 755 $$TEST $(DESTDIR)$(BINDIR)/$$TEST ; \
	done

# Distribution tarball

PACKAGEDIR = $(PACKAGE)-$(VERSION)
SRC = *.c lib/*.c include/arcom/*.h include/linux/arcom/*.h \
		java/*.java java/com/arcom/aim104/*.java \
		java/com/arcom/aim104/*.c \
		java/com/arcom/aim104/*.cc
DOCS = docs/*.sgml docs/linux-aim104-drivers.dsl docs/html/*.html \
		java/docs/*.html java/docs/com/arcom/aim104/*.html \
		java/docs/stylesheet.css java/docs/package-list
EXTRA_DIST = mkdevs.sh ael/ChangeLog ael/aim104-tests.files \
		ael/control ael/control.modules ael/j9-aim104-classes.files \
		ael/j9-aim104-libraries.files ael/libaim104-dev.files \
		ael/libaim104-0.files ael/rules

dist: docs
	rm -Rf $(PACKAGEDIR)
	for i in `find . ! -name CVS -type d -printf "%p "` ; do \
		mkdir -p $(PACKAGEDIR)/$$i ; \
	done
	for i in `find . -name Makefile -printf "%p "` ; do \
		cp $$i $(PACKAGEDIR)/$$i ; \
	done
	install Kbuild $(PACKAGEDIR)/Kbuild
	for i in $(SRC) ; do \
		cp $$i $(PACKAGEDIR)/$$i ; \
	done
	for i in $(DOCS) ; do \
		cp $$i $(PACKAGEDIR)/$$i ; \
	done
	for i in $(EXTRA_DIST) ; do \
		cp $$i $(PACKAGEDIR)/$$i ; \
	done
	tar cf - $(PACKAGEDIR) | bzip2 -9c > $(PACKAGE)_$(VERSION).orig.tar.bz2
	rm -Rf $(PACKAGEDIR)

# Misc.

clean:
	rm -f *.o core $(TEST_PROGS)
	cd lib && $(MAKE) clean
	cd java && $(MAKE) clean
	cd docs && $(MAKE) clean

dist-clean:
	rm -f *.o core $(TEST_PROGS) *~
	cd lib && $(MAKE) dist-clean
	cd java && $(MAKE) dist-clean
	cd docs && $(MAKE) dist-clean

real-clean:
	rm -f *.o core $(TEST_PROGS) *~
	cd lib && $(MAKE) real-clean
	cd java && $(MAKE) real-clean
	cd docs && $(MAKE) real-clean

endif # KERNELRELEASE != ""
