%% Extract & Plot DATA from Crazyflie Test
% This code extract and analise the data of the Crazyflie's Flyght
%%

clc; clear all; close all;
filename2=csvread('data_parametersV7.csv');
filename2(:,18)=[];
filename2 = filename2'
filename=csvread('data_positionV7.csv');
filename1=csvread('data_stabilizerV7.csv');

parameters = filename2(:,1);
parameters2 = parameters';

timestamp = filename(:,1);
x = filename(:,2);
y = filename(:,3);
z = filename(:,4);
roll = filename(:,5);
pitch = filename(:,6);
thrust = filename(:,7);
x_des = filename(:,8);
y_des = filename(:,9);
z_des = filename(:,10);
xref = filename(:,11);
yref = filename(:,12);
zref = filename(:,13);
yawref = filename(:,14);
yaw = filename(:,15);


real_pitch = -filename1(:,2);
real_roll = filename1(:,3);
real_thrust = filename1(:,4);



t = 0:0.01:(length(x)-1)*0.01;
t2 = 0:0.01:(length(real_roll)-1)*0.01;

%Analyse des courbes
if(x(1)<x_des(1))
    Sx = stepinfo(x,t,x_des(1));
else
    Sx = stepinfo(2*x_des(1)-x,t,x_des(1));
end

if(y(1)<y_des(1))
    Sy = stepinfo(y,t,y_des(1));
else
    Sy = stepinfo(2*y_des(1)-y,t,y_des(1));
end
    
if(z(1)<z_des(1))
    Sz = stepinfo(z,t,z_des(1));
else
    Sz = stepinfo(2*z_des(1)-z,t,z_des(1));
end


%Extraction des temps de réponses
t_rep_x = Sx.RiseTime;
t_rep_y = Sy.RiseTime;
t_rep_z = Sz.RiseTime;
parameters2(end+1) = t_rep_x;
parameters2(end+1) = t_rep_y;
parameters2(end+1) = t_rep_z;

%Extraction de l'overshoot
overshoot_x = Sx.Overshoot;
overshoot_y = Sy.Overshoot;
overshoot_z = Sz.Overshoot;
parameters2(end+1) = overshoot_x;
parameters2(end+1) = overshoot_y;
parameters2(end+1) = overshoot_z;

%Calcul du tracking error
TEX = (x-xref);
TEY = (y-yref);
TEZ = (z-zref);

parameters2 = parameters2';


figure(1)
plot3(x,y,z)
xlabel('x'); ylabel('y'); zlabel('z');
grid on

f2 = figure(2);
subplot(2,4,[1,2])
plot(t,x,t,x_des,t,xref)
legend('x', 'x_c_o_n_s', 'x_r_e_f');
axis([0 (length(x)-1)*0.01 0 Inf])
title('x'); xlabel('t'); ylabel('x');
grid on

subplot(2,4,[3,4])
plot(t,y,t,y_des,t,yref)
legend('y', 'y_c_o_n_s', 'y_r_e_f');
axis([0 (length(x)-1)*0.01 0 Inf])
title('y'); xlabel('t'); ylabel('y');
grid on

subplot(2,4,[5,6])
plot(t,z,t,z_des,t,zref)
legend('z', 'z_c_o_n_s', 'z_r_e_f');
axis([0 (length(x)-1)*0.01 0 Inf])
title('z'); xlabel('t'); ylabel('z');
grid on

subplot(2,4,[7,8])
plot(t,yaw,t,yawref)
legend('yaw', 'yaw_r_e_f');
axis([0 (length(x)-1)*0.01 -Inf Inf])
title('yaw'); xlabel('t'); ylabel('yaw');
grid on

f3 = figure(3);
subplot(2,4,[1,2])
plot(t,roll,t2, real_roll)
title('roll'); legend('roll', 'realRoll'); xlabel('t'); ylabel('roll');
axis([0 (length(x)-1)*0.01 -3 3])
grid on

subplot(2,4,[3,4])
plot(t,pitch, t2, real_pitch)
title('pitch'); legend('pitch', 'realPitch'); xlabel('t'); ylabel('pitch');
axis([0 (length(x)-1)*0.01 -3 3])
grid on

subplot(2,4,[6,7])
plot(t,thrust,t2,real_thrust)
axis([0 (length(x)-1)*0.01 -Inf Inf])
title('thrust'); legend('thrust', 'realThrust'); xlabel('t'); ylabel('thrust');
grid on

figure(4)
subplot(2,4,[1,2])
plot(t,TEX)
title('Tracking error X'); xlabel('t'); ylabel('TE');
grid on

subplot(2,4,[3,4])
plot(t,TEY)
title('Tracking error Y'); xlabel('t'); ylabel('TE');
grid on

subplot(2,4,[6,7])
plot(t,TEZ)
title('Tracking error Z'); xlabel('t'); ylabel('TE');
grid on
