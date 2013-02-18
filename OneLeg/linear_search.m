clc
close all
clear all

SimParam

Amp = leg_length/2;

load_system('water_hopper.mdl');
T_des = 20;

FREQ = 10:1:100;
RES = [];

for kk = 1 : numel(FREQ)
    freq = FREQ(kk);
    clc
    disp(freq)
   
    T_sim = sim('water_hopper.mdl',T_des);
    
    sink = 0;
    if(T_sim(end) < T_des),sink = 1;end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball = ball_position(:,2);
    Y_toe  = toe_position(:,2);
    Work = Work(:,2);
    force = force(:,2);
    %%%% Trimming %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball(T_sim <T_sim(end)/2) = [];
    Work(T_sim <T_sim(end)/2) = [];
    force(T_sim <T_sim(end)/2) = [];
    Y_toe(T_sim <T_sim(end)/2) = [];
    
    T_sim(T_sim <T_sim(end)/2) = [];
    T_sim = T_sim - T_sim(1);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ave = mean(Y_ball);
    Y_ball_n = Y_ball - Y_ave;
    f = fit( T_sim, Y_ball_n,'sin1' );
    ball_freq = f.b1;
    ball_osc  = f.a1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fit2 = fit( T_sim, Work,'a*x+b' );
    power = fit2.a;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Touch = .5*(1-sign(Y_toe));
    dT = [0; diff(T_sim)];
    water_time = cumsum(Touch.*dT);
    duty_ratio = water_time(end) / (T_sim(end)-T_sim(1));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    hover = 0;
    body_ratio = 1-  numel(find(sign(Y_ball)== -1))/numel(Y_ball);
    if(body_ratio > 0.95 ), hover =1;end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    result = [Y_ave ball_osc ball_freq power duty_ratio sink hover];
    
    RES = [RES ; result];
end

save

%%
Y = RES(:,1);
dY = RES(:,2);
Y_high = Y + dY;
Y_low  = Y - dY;

plot(FREQ(FREQ>33), Y(FREQ>33),'k')
hold on
plot(FREQ(FREQ>33), Y_low(FREQ>33), FREQ(FREQ>33) , Y_high(FREQ>33) , '-g')
xlim([11 100])

power = RES(:,4);
figure;
plot(FREQ(FREQ>24),power(FREQ>24),'k')

%%
freq = FREQ(2:end);
sink = RES(2:end,6);
hover = RES(2:end,7);

Ind = find(sink);
Ind1 = min(Ind);
Ind2 = max(Ind);

Ind = find(hover);
Ind3 = min(Ind);
Ind4 = max(Ind);

figure;hold on
fill([freq(Ind1) freq(Ind1) freq(Ind2) freq(Ind2)],[-.1 .1 .1 -.1],'r')
fill([freq(Ind3) freq(Ind3) freq(Ind4) freq(Ind4)],[-.1 .1 .1 -.1],'b')
fill([freq(Ind2) freq(Ind2) freq(Ind3) freq(Ind3)],[-.1 .1 .1 -.1],[0.2 0.2 0.2])

axis([0 100 -2 2])
legend('Sinking','Submerging','Hovering')
xlabel('Frequency [rad/s]')
% set(gca,'YTick',[])
%%%%%%%%%
