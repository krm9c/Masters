# Makefile for libaim104

all: lib

# Directories
PREFIX := /usr
LIBDIR := $(PREFIX)/lib
INCDIR := $(PREFIX)/include
BINDIR := $(PREFIX)/bin

# C Library

LIB_VERSION_MAJOR = 0
LIB_VERSION = $(LIB_VERSION_MAJOR).$(VERSION)
LIB_NAME = libaim104.so
LIB_SONAME = libaim104.so.$(LIB_VERSION_MAJOR)
LIB_FULL_NAME = $(LIB_NAME).$(LIB_VERSION)

OBJS = relay8.o out16.o in16.o io32.o multi-io.o

CFLAGS = -g -Wall -O2 -I../include -fPIC

lib: $(LIB_FULL_NAME)

$(LIB_FULL_NAME): $(OBJS)
	$(CC) -shared -Wl,-soname,$(LIB_SONAME) -o $(LIB_FULL_NAME) $(OBJS)
	ln -sf $(LIB_FULL_NAME) $(LIB_NAME)
	ln -sf $(LIB_FULL_NAME) $(LIB_SONAME)

relay8.o: relay8.c ../include/arcom/libaim104.h ../include/linux/arcom/aim104.h
out16.o: out16.c ../include/arcom/libaim104.h ../include/linux/arcom/aim104.h
in16.o: in16.c ../include/arcom/libaim104.h ../include/linux/arcom/aim104.h
io32.o: io32.c ../include/arcom/libaim104.h ../include/linux/arcom/aim104.h
multi-io.o: multi-io.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h

# Tests

test: test-lib

# C library tests

TEST_PROGS = test_libaim104-relay8 test_libaim104-in16 test_libaim104-out16 \
		test_libaim104-io32 test_libaim104-multi-io

test-lib: CFLAGS = -g -Wall -I../include
test-lib: LDFLAGS = -L.
test-lib: LDLIBS = -laim104

test-lib: $(TEST_PROGS)

test_libaim104-%: test_aim104-%.o
	$(CC) $(LDFLAGS) $(LDLIBS) $< -o $@

test_libaim104-relay8: test_aim104-relay8.o
test_libaim104-in16: test_aim104-in16.o
test_libaim104-out16: test_aim104-out16.o
test_libaim104-io32: test_aim104-io32.o
test_libaim104-multi-io: test_aim104-multi-io.o

test_aim104-relay8.o: test_aim104-relay8.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h
test_aim104-in16.o: test_aim104-in16.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h
test_aim104-out16.o: test_aim104-out16.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h
test_aim104-io32.o: test_aim104-io32.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h
test_aim104-multi-io.o: test_aim104-multi-io.c ../include/arcom/libaim104.h \
		../include/linux/arcom/aim104.h

# Install
.PHONE: install
install: lib test
	install -D $(LIB_FULL_NAME) $(DESTDIR)$(LIBDIR)/$(LIB_FULL_NAME)
	(cd $(DESTDIR)$(LIBDIR) && \
		ln -sf $(LIB_FULL_NAME) $(LIB_NAME) && \
		ln -sf $(LIB_FULL_NAME) $(LIB_SONAME) )
	install -D ../include/arcom/libaim104.h $(DESTDIR)$(INCDIR)/arcom/libaim104.h
	install -D ../include/linux/arcom/aim104.h $(DESTDIR)$(INCDIR)/linux/arcom/aim104.h
	install -d $(DESTDIR)$(BINDIR)
	set -e ; for TEST in $(TEST_PROGS) ; do \
		install -m 755 $${TEST} $(DESTDIR)$(BINDIR)/$${TEST} ; \
	done

# Misc.

clean:
	rm -f $(LIB_FULL_NAME) $(LIB_SONAME) $(LIB_NAME) *.o core \
		$(TEST_PROGS)

dist-clean:
	rm -f $(LIB_FULL_NAME) $(LIB_SONAME) $(LIB_NAME) *.o core *~ \
		$(TEST_PROGS)

real-clean:
	rm -f $(LIB_FULL_NAME) $(LIB_SONAME) $(LIB_NAME) *.o core *~ \
		$(TEST_PROGS)
