# Makefile for Aim104 documentation

all:

# Documentation

JADE = jade
JADEFLAGS =

docs: html/index.html

html/index.html: *.sgml linux-aim104-drivers.dsl
	rm -f html/*.html
	$(JADE) $(JADEFLAGS) -t sgml linux-aim104-drivers.sgml

# Misc.

clean:

dist-clean:
	rm -f *~

real-clean:
	rm -f html/*.html
