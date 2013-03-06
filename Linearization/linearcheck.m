w1 = 0:100;
w2 = 0:100;

w1c = [0 100];
w2c = [0 100];
[W1, W2] = meshgrid(w1,w2);
[W1c, W2c] = meshgrid(w1c,w2c);
a = 1/16;

F1 = (W1.^2.*W2+ a*W1.*W2.^2)./(W1+ W2);
F2 = (W2 + a*W1)./(W1 + W2);

while 1
	k1 = randi(100);
	k2 = randi(100);
	w10 = w1(k1);
	w20 = w2(k2);

	L1 = (w10^2*w20 + a*w10 *w20^2)/(w10 + w20) + (w20*(w10^2 + 2*w10*w20 + a*w20^2))/((w10 + w20)^2) * (W1c - w10) ...
		+ (w10*(w10^2 + 2*a*w10*w20 + a*w20^2))/((w10 + w20)^2)*(W2c - w20);
	L2 = (w20 + a*w10)/(w10 + w20) + (a-1)*w20/((w10 + w20)^2) * (W1c - w10) + ((1-a)*w10)/((w10 + w20)^2)*(W2c - w20);

	figure(1)
	surf(W1,W2,F1)
	hold on	
	surf(W1c,W2c,L1)
	plot3(w10,w20,(w10^2*w20 + a*w10 *w20^2)/(w10 + w20),'o','MarkerFaceColor','r','MarkerEdgeColor','r','MarkerSize',10)
	hold off
	axis([0 100 0 100 min(min(F1)) max(max(F1))])
	xlabel('W1')
	ylabel('W2')
	zlabel('F1')

	figure(2)
	surf(W1,W2,F2)
	hold on
	surf(W1c,W2c,L2)
	plot3(w10,w20,(w20 + a*w10)/(w10 + w20),'o','MarkerFaceColor','r','MarkerEdgeColor','r','MarkerSize',10)
	hold off
	axis([0 100 0 100 min(min(F2)) max(max(F2))])
	xlabel('W1')
	ylabel('W2')
	zlabel('F2')
	view([50 10])
	pause(1)
end


