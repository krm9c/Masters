# Makefile for Java Aim104 library.

all: lib-java

install: install-lib-java

# Java library

JAVA_LIB_VERSION_MAJOR=1
JAVA_LIB_VERSION = $(JAVA_LIB_VERSION_MAJOR).$(VERSION)
JAVA_LIB_NAME=libaim104_java.so
JAVA_LIB_SONAME=$(JAVA_LIB_NAME).$(JAVA_LIB_VERSION_MAJOR)
JAVA_LIB_FULL_NAME = $(JAVA_LIB_NAME).$(JAVA_LIB_VERSION)

JAVAC = javac
JAVAH = javah
JAVA_INCLUDES = -I$(JAVA_HOME)/include -I$(JAVA_HOME)/include/linux
CFLAGS = $(JAVA_INCLUDES) -I../../../../include -fPIC -Wall -g -O2

%.class: %.java
	$(JAVAC) -classpath ../../.. $<

com_arcom_aim104_%.h: %.class
	$(JAVAH) -jni -classpath ../../.. com.arcom.aim104.$(basename $<)

lib-java: $(JAVA_LIB_FULL_NAME) Aim104.class Aim104Relay8.class \
	Aim104In16.class Aim104Out16.class Aim104IO32.class \
	Aim104MultiIO.class \
	SysIOException.class

$(JAVA_LIB_FULL_NAME): Aim104_jni.o Aim104Relay8_jni.o Aim104Out16_jni.o \
		Aim104IO32_jni.o Aim104MultiIO_jni.o
	$(CC) -o $@ -shared -Wl,-soname,$(JAVA_LIB_SONAME) $^

Aim104_jni.o: Aim104_jni.c com_arcom_aim104_Aim104.h
Aim104Relay8_jni.o: Aim104Relay8_jni.c com_arcom_aim104_Aim104Relay8.h
Aim104Out16_jni.o: Aim104Out16_jni.c com_arcom_aim104_Aim104Out16.h
Aim104IO32_jni.o: Aim104IO32_jni.c com_arcom_aim104_Aim104IO32.h
Aim104MultiIO_jni.o: Aim104MultiIO_jni.c com_arcom_aim104_Aim104MultiIO.h

Aim104.class: Aim104.java SysIOException.class
Aim104Relay8.class: Aim104Relay8.java SysIOException.class
Aim104In16.class: Aim104In16.java SysIOException.class
Aim104Out16.class: Aim104Out16.java SysIOException.class
Aim104IO32.class: Aim104IO32.java SysIOException.class
Aim104MultiIO.class: Aim104MultiIO.java SysIOException.class

install-lib-java:
	install -D $(JAVA_LIB_FULL_NAME) $(DESTDIR)$(WSDDBINDIR)/$(JAVA_LIB_FULL_NAME)
	ln -sf $(JAVA_LIB_FULL_NAME) $(DESTDIR)$(WSDDBINDIR)/$(JAVA_LIB_SONAME)
	ln -sf $(JAVA_LIB_SONAME) $(DESTDIR)$(WSDDBINDIR)/$(JAVA_LIB_NAME)

# GCJ library

GCJ_LIB_VERSION_MAJOR = 0
GCJ_LIB_VERSION = $(GCJ_LIB_VERSION_MAJOR).0.0
GCJ_LIB_NAME=libaim104_gcj.so
GCJ_LIB_SONAME=$(GCJ_LIB_NAME).$(GCJ_LIB_VERSION_MAJOR)
GCJ_LIB_FULL_NAME = $(GCJ_LIB_NAME).$(GCJ_LIB_VERSION)

GCJ = gcj
GCJH = gcjh
GCJ_INCLUDES = -I/opt/gcc-3.0/include
CXXFLAGS = $(GCJ_INCLUDES) -I../../.. -I../../../../include -fPIC -Wall -g -O2

%.h: %.class
	$(GCJH) -d ../../.. -o $@ $(basename $<)

%.o: %.class
	$(GCJ) -I../../.. -o $@ -c $<

lib-gcj: $(GCJ_LIB_FULL_NAME)

$(GCJ_LIB_FULL_NAME): Aim104_cni.o Aim104Relay8_cni.o Aim104Out16_cni.o \
		Aim104IO32_cni.o Aim104MultiIO_cni.o \
		Aim104.o Aim104Relay8.o Aim104In16.o Aim104Out16.o \
		Aim104IO32.o Aim104MultiIO.o \
		SysIOException.o
	$(CC) -o $@ -shared -Wl,-soname,$(GCJ_LIB_SONAME) $^
	ln -sf $(GCJ_LIB_FULL_NAME) $(GCJ_LIB_SONAME)
	ln -sf $(GCJ_LIB_FULL_NAME) $(GCJ_LIB_NAME)

Aim104_cni.o: Aim104_cni.cc Aim104.h
Aim104Relay8_cni.o: Aim104Relay8_cni.cc Aim104Relay8.h
Aim104Out16_cni.o: Aim104Out16_cni.cc Aim104Out16.h
Aim104IO32_cni.o: Aim104IO32_cni.cc Aim104IO32.h
Aim104MultiIO_cni.o: Aim104MultiIO_cni.cc Aim104MultiIO.h

# Misc.

clean:
	rm -f *.class *.o lib*.so* *.h core

dist-clean:
	rm -f *.class *.o lib*.so* *.h core *~

real-clean:
	rm -f *.class *.o lib*.so* *.h core *~
