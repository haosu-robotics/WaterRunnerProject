all: report.pdf report_short.pdf

report_short.pdf : report_short.tex 4bar.pdf FB.pdf
	pdflatex report_short.tex
	bibtex report_short
	pdflatex report_short.tex
	pdflatex report_short.tex

report.pdf : report.tex 4bar.pdf FB.pdf
	pdflatex report.tex
	bibtex report
	pdflatex report.tex
	pdflatex report.tex
