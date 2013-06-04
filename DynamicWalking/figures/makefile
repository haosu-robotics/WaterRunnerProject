all: cpg-crop.pdf cpg.pdf foot_traj_orig.eps foot_traj_simple.eps wheuristic.pdf woheuristic.pdf

cpg-crop.pdf : cpg.pdf
	pdfcrop cpg.pdf

cpg.pdf : cpg.tikz cpg.tex ../mycolors.tex
	xelatex cpg.tex

foot_traj_orig.eps : foot_traj_orig.graffle leg.pdf legspeed.pdf
	./graffle.sh foot_traj_orig.graffle foot_traj_orig.eps

foot_traj_simple.eps : foot_traj_simple.graffle leg2.pdf leg2speed.pdf
	./graffle.sh foot_traj_simple.graffle foot_traj_simple.eps

leg.pdf legspeed.pdf leg2.pdf leg2speed.pdf : traj.py matplotlibrc
	python traj.py

wheuristic.pdf woheuristic.pdf : results.py data.mat
	python results.py
