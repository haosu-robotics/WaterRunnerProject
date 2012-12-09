clc
close all

%% define lengthes in four bar mechanism

ll_Body = 0;
ll_l1 = 0.0615;
ll_l2 = 0.0218;
ll_l3 = 0.0748;
ll_l4 = 0.0468;
ll_l5 = 0.0624;

check = [ll_l1 ll_l2 ll_l3 ll_l4];
flag = sum(check)- 2*(min(check) + max(check));

if flag > 0;
    disp('four bar mechanism constraint is satisfied')
else
        disp('four bar mechanism constraint is NOT satisfied')
        
end



P(1) = 4*ll_l1^2+4*ll_l4^2;
P(2) = -4*ll_l1^2*ll_l4+4*ll_l4*ll_l3^2-4*ll_l4^3-4*ll_l4*ll_l2^2;
P(3) = ll_l2^4+ll_l1^4-2*ll_l1^2*ll_l2^2+ll_l3^4+2*ll_l1^2*ll_l4^2-2*ll_l1^2*ll_l3^2-2*ll_l3^2*ll_l2^2-2*ll_l4^2*ll_l3^2+2*ll_l4^2*ll_l2^2+ll_l4^4;
RO = roots(P);
XX = (RO.*(2*ll_l4)  -ll_l4^2+ll_l3^2+ll_l1^2-ll_l2^2)./2./ll_l1; 
YY = RO;

ind = 1;
XX = XX(ind);
YY = YY(ind);

XX = ll_Body + XX;
%%% setting simmechanics parameters

%%%%%%%%%%%%%%% L1 %%%%%%%%%%%%%%%%%%%%%%
L1_CS1 = [0 0 0];
L1_CS2 = [-ll_Body 0 0];
L1_CS3 = [-ll_Body-ll_l1 0 0];
L1_l = ll_Body + ll_l1;
L1_CG = [-L1_l/2 0 0];
L1_m = 0.5;
L1_inertia = 1/12*L1_m*L1_l^2;

%%%%%%%%%%%%%%% L2 %%%%%%%%%%%%%%%%%%%%%%
L2_CS1 = [-L1_l 0 0];
L2_CS2 = [-XX YY 0];
L2_CG = (L2_CS1 + L2_CS2)./2;
L2_m = 0.1;
L2_inertia = 1/12*L2_m*ll_l2^2;

%%%%%%%%%%%%%%% L3 %%%%%%%%%%%%%%%%%%%%%%
L3_CS1 = [-ll_Body ll_l4 0];
L3_CS2 = [-XX YY 0];
extension = (ll_l5+ll_l3)/ll_l3;
L3_CS3 = L3_CS1 + extension.*(L3_CS2-L3_CS1);
L3_CG = (L3_CS1 + L3_CS3)./2;
L3_m = 0.1;
L3_inertia = 1/12*L2_m*ll_l2^2;

%%%%%%%%%%%%%%% L4 %%%%%%%%%%%%%%%%%%%%%%
L4_CS1 = [-ll_Body 0 0];
L4_CS2 = [-ll_Body ll_l4 0];
L4_CG  = [-ll_Body ll_l4/2 0];
L4_m = 0.1;
L4_inertia = 1/12*L4_m*ll_l4^2;


%%%%% draw simple lines %%%%%%%%%

figure;
hold on
plot([L1_CS1(1) L1_CS3(1)],[L1_CS1(2) L1_CS3(2)],'r')
plot([L2_CS1(1) L2_CS2(1)],[L2_CS1(2) L2_CS2(2)],'g')
plot([L3_CS1(1) L3_CS3(1)],[L3_CS1(2) L3_CS3(2)],'b')
plot([L4_CS1(1) L4_CS2(1)],[L4_CS1(2) L4_CS2(2)],'k')
 axis equal
%%
l3_angle_0 = atan2(ll_l4-YY,XX-ll_Body);
water_level = -0.03;
