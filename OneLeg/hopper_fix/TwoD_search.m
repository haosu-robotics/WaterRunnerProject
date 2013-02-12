clc
close all
clear all

SimParam

load_system('water_hopper_fix.mdl');
T_des = 20;

h0 = leg_length;

lin1 = linspace(20,100,10);
lin2 = linspace(leg_length/10,leg_length,10);
[FREQ,AMP] = meshgrid(lin1,lin2);
POWER  = zeros(size(FREQ));
FORCE  = zeros(size(FREQ));


for kk = 1 : size(FREQ,2)
    for jj = 1 : size(AMP,1)
        
        freq = FREQ(1,kk);
        Amp  = AMP(jj,1);
        
        clc
        disp([freq Amp])
        
        T_sim = sim('water_hopper_fix.mdl',T_des);
        
        Work = Work(:,2);
        force = Int_Force(:,2);
        Work(T_sim <T_sim(end)/2) = [];
        force(T_sim <T_sim(end)/2) = [];
        T_sim(T_sim <T_sim(end)/2) = [];
        T_sim = T_sim - T_sim(1);
        
        fit2 = fit( T_sim, Work,'a*x+b' );
        power = fit2.a;
        
        fit3 = fit( T_sim, force,'a*x+b' );
        force = fit3.a;
        
        POWER(kk,jj) = power;
        FORCE(kk,jj) = force;
        
    end
end

save


