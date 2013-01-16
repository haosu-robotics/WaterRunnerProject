V  = 40;
duty_factor = 0.5;

% V = speed;
alpha = 0.05;    % transition time factor

D = 2*pi;   % one cycle
T = D/V;    % period time

k1 =    duty_factor  - alpha;
k2 = (1-duty_factor) - alpha;

T1 = T * k1;
T2 = T * k2;
Ts = T * alpha;

% disp(T)
% disp(T1+T2+2*Ts)

MM(1,:) = [duty_factor         1-duty_factor];
MM(2,:) = [2*k1+alpha          alpha        ];
% MM(3,:) = [alpha            2*(k2+1.5*alpha)];

V1V2 = MM\[V;V];
V1 = V1V2(1);
V2 = V1V2(2);

Amp_Gen1 = (V2-V1)/Ts;
Amp_Gen2 = (V1-V2)/Ts;
Del_Gen1 = T1;
Del_Gen2 = T1+Ts+T2;

Per_Gen = T;
DF_Gen  = Ts/T;

% dt = 0.001;
% time = 0:dt:T;
% signal = zeros(size(time));
% 
% signal(time>T1)          = (V2-V1)/Ts;
% signal(time>T1+Ts)       = 0;
% signal(time>T1+Ts+T2)    = (V1-V2)/Ts;
% signal(time>T1+Ts+T2+Ts) = 0;
% 
% sig2 = V1+ cumsum(signal).*dt;
% sig3 = cumsum(sig2).*dt;
% 
% figure;
% subplot(311)
% plot(time,signal)
% grid on
% 
% subplot(312)
% plot(time,sig2,'r')
% grid on
% 
% subplot(313)
% plot(time,sig3,'r')
% grid on
% 
% pause(2)
% close 
% 





% simParams





