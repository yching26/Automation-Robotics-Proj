clc;clear;

%Design Parameters
scale = 1/2;
l = [.30 .32 .08]*scale;    %link lengths [m]
p = 2.7e3;            %link material density (Al) [kg/m^3]
p = 1.5e3;            %>wood, ~polymer
t = 1.5e-3;             %link thickness [m]

wd = 0.7e-3;    %wire diameter
wireTScon = 600e6; %[Pa] Constantan wire tensile strength (455e6 - 860e6) [Pa]
wireTScop = 220e6; %[Pa] Copper wire tensile strength
wireF = wireTScop*pi*wd^2/4;   %wire required shear force
wireT = .0254*wireF;           %torque by clipper

% 18 AWG wire cutter: https://www.homedepot.com/p/Milwaukee-4-75-in-Mini-Flush-Cutters-48-22-6105/305760837
% length: 5'', width: 0.5'', height: 2''
% weight: 0.13 lb
wcutter = 0.13*0.45359237;
wstepper = 0.03719; %[kg] stepper weight
%motor3 [.28kg, 140mNm]
%https://www.digikey.com/product-detail/en/nmb-technologies-corporation/BLDC40P30A/BLDC40P30A-ND/6021443

wm = [.3 .3 0.018]; %[bl,bl,bl]
we = wcutter+wstepper;  %end effector weight
%Me = wireT;             %required cutting torque
scale2 = 1;
Me = 0.3*scale2;              %stepper motor max torque
%---------------------------------

Fe = we;
g = 9.81;       
w = p*l*t*g;    %link weights

A = [0 0 0 1 -1 0;...
    1 -1 0 -l(1) -l(1) 0;...
    0 0 0 0 1 -1;...
    0 1 -1 0 -l(2) -l(2);...
    0 0 0 0 0 1;...
    0 0 1 0 0 -l(3)];

b = [wm(1)+w(1);...
    -l(1)*wm(1);...
    wm(2)+w(2);...
    -l(2)*wm(2);...
    Fe+wm(3)+w(3);...
    Me + l(3)*(Fe-wm(3))];

x = A^(-1)*b;
sprintf('max static motor torques: %.2f\n',x(1:3))
sprintf('max static reaction forces on joints: %.2f\n',x(4:end))
    



