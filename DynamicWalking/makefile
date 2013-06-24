all: .PHONY poster.pdf

figs := RI_large.jpg cpg-crop.pdf schematic-crop.pdf foot_traj_orig.eps foot_traj_simple.eps wheuristic.pdf woheuristic.pdf robot.JPG basilisk-lizard.jpg
figs := $(figs:%=figures/%)

.PHONY : 
	make -C figures

poster.pdf : poster.tex beamerthemeNST.sty mycolors.tex intro.tex modeling.tex control.tex results.tex results2.tex acknowledgement.tex conclusion.tex $(figs)
	xelatex poster.tex
