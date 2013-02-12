clc
close all
clear all

SimParam

load_system('water_hopper.mdl');
T_des = 20;


lin1 = linspace(20,100,20);
lin2 = linspace(leg_length/10,leg_length,20);
[FREQ,AMP] = meshgrid(lin1,lin2);
Y_BALL = zeros(size(FREQ));
Y_OSC  = zeros(size(FREQ));
POWER  = zeros(size(FREQ));
SUBFA  = zeros(size(FREQ));
SINK   = zeros(size(FREQ));
HOVER  = zeros(size(FREQ));




for kk = 1 : size(FREQ,2)
    for jj = 1 : size(AMP,1)
        
        freq = FREQ(1,kk);
        Amp  = AMP(jj,1);
        
        clc
        disp([freq Amp])
        
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
        
        Y_BALL(kk,jj) = Y_ave;
        Y_OSC(kk,jj) = ball_osc;
        POWER(kk,jj) = power;
        SUBFA(kk,jj) = duty_ratio ;
        SINK(kk,jj) = sink;
        HOVER(kk,jj) = hover;
        
    end
end

save
%%

mesh(FREQ,AMP,POWER)

xx = reshape(Y_BALL,1,[]);
yy = reshape(POWER,1,[]);
tt = reshape(SINK,1,[]);

yy(find(xx>2)) = [];
tt(find(xx>2)) = [];
xx(find(xx>2)) = [];

xxx = xx(tt == 0);
yyy = yy(tt == 0);

figure;
plot(xxx,yyy,'x')
% xlim([-.2 .2])

%%


