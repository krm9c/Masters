

#!/bin/bash

#texfile="dissertation_publication"
texfile="Krishnan_Raghavan_Thesis_MS"
#texfile="ptotemplate"

rm -f *.{aux,bbl,blg,dvi,glg,glo,gls,idx,ist,fff,nlo,nls,ilg}
rm -f *.{lof,log,lot,out,toc,pdf,ps,ttt,xdy}


pdflatex $texfile

bibtex $texfile

pdflatex $texfile

pdflatex $texfile

#Remove these two line if yuu want to see the temporary files after compilation

rm -f *.{aux,bbl,blg,dvi,glg,glo,gls,idx,ist,fff,nlo,nls,ilg}
rm -f *.{lof,log,lot,out,toc,ps,ttt,xdy}