# Makefile for Java library and test programs.

all: lib-java

install: install-lib-java

# Java library

lib-java: lib-java2 aim104.jar

lib-java2:
	cd com/arcom/aim104 && $(MAKE) lib-java

aim104.jar: com/arcom/aim104/*.java \
		$(patsubst %.java,%.class,$(wildcard com/arcom/aim104/*.java))
	jar cf aim104.jar $^

install-lib-java:
	cd com/arcom/aim104 && $(MAKE) install-lib-java
	install -D aim104.jar $(DESTDIR)$(WSDDLIBDIR)/aim104.jar

# GCJ library

lib-gcj:
	cd com/arcom/aim104 && $(MAKE) lib-gcj

# Tests

test: test-java test-jxe

# Java tests

JAVAC = javac

%.class: %.java
	$(JAVAC) -classpath . $<

test-java: Test_Aim104Relay8.class Test_Aim104In16.class \
		Test_Aim104Out16.class Test_Aim104IO32.class \
		Test_Aim104MultiIO.class

# J9 JXE tests

IVEHOME := /opt/DeviceDeveloper/wsdd5.0/ive
JXELINK := jxelink
JXELINKFLAGS := -cp $(IVEHOME)/lib/jclFoundation/classes.zip -cp . \
	-removeUnused -noDots -noLogo

%.jxe: %.class
	$(JXELINK) $(JXELINKFLAGS) -includeMainMethod $(basename $<) $<

test-jxe: Test_Aim104Relay8.jxe Test_Aim104In16.jxe Test_Aim104Out16.jxe \
		Test_Aim104IO32.jxe Test_Aim104MultiIO.jxe

# GCJ tests

TEST_PROGS = Test_Aim104Relay8 Test_Aim104In16 Test_Aim104Out16 \
	Test_Aim104IO32 Test_Aim104MultiIO

GCJ = gcj

%.o: %.class
	$(GCJ) -o $@ -c $<

%: %.o
	$(GCJ) -o $@ -Lcom/arcom/aim104 --main=$(basename $<) \
		$^ -laim104_gcj -lstdc++

test-gcj: $(TEST_PROGS)

Test_Aim104Relay8: Test_Aim104Relay8.o
Test_Aim104In16: Test_Aim104In16.o
Test_Aim104Out16: Test_Aim104Out16.o
Test_Aim104IO32: Test_Aim104IO32.o
Test_Aim104MultiIO: Test_Aim104MultiIO.o

# Documentation

JAVADOC = javadoc
JAVADOCFLAGS = -package -windowtitle "Aim104 library" \
        -header "Aim104 library" -footer "Aim104 library"

docs: docs/index.html

docs/index.html: com/arcom/aim104/*.java
	$(JAVADOC) -d docs -classpath . $(JAVADOCFLAGS) com.arcom.aim104

# Misc.

clean:
	rm -f $(TEST_PROGS) *.class *.o *.log *.sym *.jxe aim104.jar core
	cd com/arcom/aim104 && $(MAKE) clean

dist-clean:
	rm -f $(TEST_PROGS) *.class *.o *.log *.sym *.jxe aim104.jar core *~
	cd com/arcom/aim104 && $(MAKE) dist-clean

real-clean:
	rm -f $(TEST_PROGS) *.class *.o *.log *.sym *.jxe aim104.jar core *~
	rm -Rf docs/*.html docs/*.css docs/package-list docs/com
	cd com/arcom/aim104 && $(MAKE) real-clean
