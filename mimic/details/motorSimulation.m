clc;close all;
M = 5.8967008;
V = 860.425*1022.35*11.1125;

p = M/V;                    %density [kg/mm^3]
v = 30*30*11.1125-4^2*pi/4; %volume of block [mm^3]
m = v*p;                    %mass of block [kg]
Jb = 1/6*m*0.03^2;          %Polar moment of inertia of block [kg m^2]
km1 = 0.1287;               %Km of motor without block [rev/volt]
km2 = 0.1502;               %Km of motor with block [rev/volt]
Tm1 = 0.00886;              %Tm of motor without block [s]
Tm2 = 0.008367;             %Tm of motor with block [s]
Jm = Tm1/(Tm2-Tm1)*Jb;      %J of the motor without block [kg m^2]

L = .35;    %length of bar
w = .02;    %width of bar
t = 0.01;   %thickness of bar
p = 1;      %density of bar
mb = p*L*t*w;   %mass of bar
J_bar = 1/12*mb*(4*L^2+w^2);    %polar moment of bar [Kg m^2]

J = Jb;                     %J of system    [Kg m^2]
Tm = Tm1*(Jm+J)/Jm;         %Tm of system   [seconds]
Km = km1*2*pi;                   %Km of system   [rad/s]

Kd = 0; %derivative gain
Kp = 1/(4*Km*Tm); %proportional gain
Ki = 0; %integral gain
%Ki = Kp^2/4/(Kd+1/Km);
%0.05 5 .05
%T = tf([Kd Kp Ki]*Km/Tm, [1 (Kd*Km+1)/Tm (Kp*Km/Tm) (Ki*Km/Tm)]);

T = tf(Km*Kp/Tm, [1 1/Tm Kp*Km/Tm]);
step(T,0.2)
hold on

Vdd = 12.24;
Kp_1 = Kp/(Vdd/100);    %controller gain
Kp_2 = 12.24/100;       %[duty cycle -> Vin] gain

