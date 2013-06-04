all: .PHONY poster.pdf

figs := cpg-crop.pdf cpg.pdf foot_traj_orig.eps foot_traj_simple.eps wheuristic.pdf woheuristic.pdf
figs := $(figs:%=figures/%)

.PHONY : 
	make -C figures

poster.pdf : poster.tex beamerthemeNST.sty mycolors.tex intro.tex modeling.tex control.tex results.tex results2.tex acknowledgement.tex conclusion.tex $(figs)
	xelatex poster.tex
