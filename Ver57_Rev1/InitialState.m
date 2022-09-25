    
%clear all, close all, clc

% Modelo en Cuaterniones
% Auth: Angel Daniel Velazquez Rodriguez (2022)

% Carga de datos de la aeronave:
% Boeing737_Data


%% Condiciones Iniciales
h = 7350; %[m]

u_ini = 0;%0.01;
v_ini = 0;
w_ini = 0;

p_ini = 0;
q_ini = 0;
r_ini = 0;

phi_ini = 0*pi/180;
theta_ini = 1*pi/180;
psi_ini = 0*pi/180;

x_ini = 0;
y_ini = 0;
z_ini = h;

Aileron_Trim = 0;
Rudder_Trim = 0;
Elevator_Trim = 0;
Engine_Trim = 0;

% %% Estado Inicial
% clc
% fprintf('Estado Inicial:\n\n')
% fprintf('\tPosición Global:\n\t x_ini =\t %g \t[m]\n\t y_ini =\t %g \t[m]\n\t z_ini =\t %g \t[m]\t{%g\t[ft]}\n\n',x_ini,y_ini,z_ini,z_ini*3.28)
% fprintf('\t phi_ini =\t\t %g \t[rad]\t{%g\t[deg]}\n\t theta_ini =\t %g \t[rad]\t{%g\t[deg]}\n\t psi_ini =\t\t %g \t[rad]\t{%g\t[deg]}\n\n',phi_ini,phi_ini*180/pi,theta_ini,theta_ini*180/pi,psi_ini,psi_ini*180/pi)
% fprintf('\tVelocidad Ejes Cuerpo:\n\t u_ini =\t %g \t[m/s]\n\t v_ini =\t %g \t[m/s]\n\t w_ini =\t %g \t[m/s]\n\n',u_ini,v_ini,w_ini)
% fprintf('\tVelocidad Angular Ejes Cuerpo:\n\t p_ini =\t %g \t[rad/s]\n\t q_ini =\t %g \t[rad/s]\n\t r_ini =\t %g \t[rad/s]\n\n',p_ini,q_ini,r_ini)
% fprintf('\tTrimado Actuadores:\n\t Aileron =\t %g \t[deg]\n\t Rudder =\t %g \t[deg]\n\t Elevator =\t %g \t[deg]\n\t Engine =\t %g \t[%%]\n',Aileron_Trim*Max_Aileron*180/pi,Rudder_Trim*Max_Rudder*180/pi,Elevator_Trim*Max_Elevator*180/pi,Engine_Trim*100)


