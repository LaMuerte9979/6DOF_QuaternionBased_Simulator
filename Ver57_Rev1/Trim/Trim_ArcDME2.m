% clear all, close all, clc

% Auth: Angel Daniel Velazquez Rodriguez (2022)

% Boeing737_Data
% 
% InitialState

Modelo = 'Trim__SimulatorModel'
% load_system(Modelo)
% set_param(Modelo,'FastRestart','on');

%% Trimmed Data
tic
precision = 1;
control__total = 0;

warning('off','all');
warning

time = 1;

%% Trimed

while control__total == 0;
    
%%% Body Velocity Calculation
UVW_trim = [320,0,0];
% quat = quaternion([phi_ini,-theta_ini,-psi_ini],'rotvecd'); %version en [deg]
quat = quaternion([-phi_ini,-theta_ini,-psi_ini],'rotvec'); %version en [rad]
quat = normalize(quat);
quat_vect = compact(quat);

uvw_trim2 = quatrotate(quat_vect,UVW_trim);

u_ini_trim1 = 0;
u_ini_trim2 = uvw_trim2(1);

v_ini_trim1 = 0;
v_ini_trim2 = uvw_trim2(2);

w_ini_trim1 = 0;
w_ini_trim2 = uvw_trim2(3);

clear quat quat_vect uvw_trim2 UVW_trim


u_ini = u_ini_trim1;
assignin('base','u_ini',u_ini)
v_ini = v_ini_trim1;
assignin('base','v_ini',v_ini)
w_ini = w_ini_trim1;
assignin('base','w_ini',w_ini)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fz_trim1 = Simulation.Global_Forces(time+1,3);
clc, clear Simulation

u_ini = u_ini_trim2;
assignin('base','u_ini',u_ini)
v_ini = v_ini_trim2;
assignin('base','v_ini',v_ini)
w_ini = w_ini_trim2;
assignin('base','w_ini',w_ini)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fz_trim2 = Simulation.Global_Forces(time+1,3);
clc, clear Simulation


control__u_ini = 0;
while control__u_ini == 0

aux_trim1 = 1-abs(Fz_trim1)/abs(Fz_trim2-Fz_trim1);
aux_trim2 = 1-abs(Fz_trim2)/abs(Fz_trim2-Fz_trim1);
u_ini_trim_med = sqrt(u_ini_trim1*u_ini_trim1*aux_trim1 + u_ini_trim2*u_ini_trim2*aux_trim2);
% w_ini_trim_med = sqrt(w_ini_trim1*w_ini_trim1*aux_trim1 + w_ini_trim2*w_ini_trim2*aux_trim2);

quat = quaternion([-phi_ini,theta_ini,psi_ini],'rotvec');
quat = normalize(quat);
quat_vect = compact(quat);
UVW_aux = quatrotate(quat_vect,[u_ini_trim_med 0 0]);

u_ini = u_ini_trim_med;
assignin('base','u_ini',u_ini)
% w_ini = -w_ini_trim_med;
w_ini = -UVW_aux(3);
clear UVW_aux
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fz_trim_med = Simulation.Global_Forces(time+1,3);
clc, clear Simulation

    if abs(Fz_trim_med) < precision
        control__u_ini = 1;
      elseif Fz_trim_med*Fz_trim1<0
        u_ini_trim2 = u_ini_trim_med;
%         w_ini_trim2 = w_ini_trim_med;
        Fz_trim2 = Fz_trim_med;
      elseif Fz_trim_med*Fz_trim2<0
        u_ini_trim1 = u_ini_trim_med;
%         w_ini_trim1 = w_ini_trim_med;
        Fz_trim1 = Fz_trim_med;
      end

end

quat = quaternion([phi_ini,theta_ini,psi_ini],'rotvec');
quat = normalize(quat);
quat_vect = compact(quat);
UVW_aux = quatrotate(quat_vect,[u_ini_trim_med 0 0]);

u_ini = u_ini_trim_med
assignin('base','u_ini',u_ini)
w_ini = -UVW_aux(3)
assignin('base','w_ini',w_ini)
% w_ini = -w_ini_trim_med

clear aux_trim1 aux_trim2 UVW_aux;
clear quat quat_vect;

%%% Engine Lever Calculation

lever_trim1 = 0;
lever_trim2 = 1;

Engine_Trim = lever_trim1;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fx_trim1 = Simulation.Global_Forces(time+1,1);
clc, clear Simulation

Engine_Trim = lever_trim2;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fx_trim2 = Simulation.Global_Forces(time+1,1);
clc, clear Simulation

control__engine = 0
while control__engine == 0
aux_trim1 = 1-abs(Fx_trim1)/abs(Fx_trim2-Fx_trim1);
aux_trim2 = 1-abs(Fx_trim2)/abs(Fx_trim2-Fx_trim1);
lever_trim_med = (lever_trim1*aux_trim1 + lever_trim2*aux_trim2);

Engine_Trim = lever_trim_med;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fx_trim_med = Simulation.Global_Forces(time+1,1);
clc, clear Simulation

    if abs(Fx_trim_med) < precision
        control__engine = 1;
      elseif Fx_trim_med*Fx_trim1<0
        lever_trim2 = lever_trim_med;
        Fx_trim2 = Fx_trim_med;
      elseif Fx_trim_med*Fx_trim2<0
        lever_trim1 = lever_trim_med;
        Fx_trim1 = Fx_trim_med;
      end
    
end

Engine_Trim = lever_trim_med
assignin('base','Engine_Trim',Engine_Trim)
clear aux_trim1 aux_trim2


%%% Elevator Calculation

deltae_trim1 = -1;
deltae_trim2 = 1;

Elevator_Trim = deltae_trim1;
assignin('base','Elevator_Trim',Elevator_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
My_trim1 = Simulation.Body_Moments(time+1,2);
clc, clear Simulation

Elevator_Trim = deltae_trim2;
assignin('base','Elevator_Trim',Elevator_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
My_trim2 = Simulation.Body_Moments(time+1,2);
clc, clear Simulation

control__deltae = 0;
while control__deltae == 0
    
aux_trim1 = 1-abs(My_trim1)/abs(My_trim2-My_trim1);
aux_trim2 = 1-abs(My_trim2)/abs(My_trim2-My_trim1);
deltae_trim_med = (deltae_trim1*aux_trim1 + deltae_trim2*aux_trim2);

Elevator_Trim = deltae_trim_med;
assignin('base','Elevator_Trim',Elevator_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
My_trim_med = Simulation.Body_Moments(time+1,2);
clc, clear Simulation

    if abs(My_trim_med) < precision
        control__deltae = 1;
      elseif My_trim_med*My_trim1<0
        deltae_trim2 = deltae_trim_med;
        My_trim2 = My_trim_med;
      elseif My_trim_med*My_trim2<0
        deltae_trim1 = deltae_trim_med;
        My_trim1 = My_trim_med;
      end

end

Elevator_Trim = deltae_trim_med
assignin('base','Elevator_Trim',Elevator_Trim)
clear aux_trim1 aux_trim2
clc


%%% Phi Calculation

phi_ini_trim1 = 0*pi/180;
phi_ini_trim2 = 80*pi/180 * Direc;

phi_ini = phi_ini_trim1;
assignin('base','phi_ini',phi_ini)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fy_trim1 = Simulation.Global_Forces(time+1,2);
U_inf_1 = sqrt(Simulation.Global_Velocity(time+1,1)^2+Simulation.Global_Velocity(time+1,2)^2);
clc, clear Simulation

phi_ini = phi_ini_trim2;
assignin('base','phi_ini',phi_ini)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fy_trim2 = Simulation.Global_Forces(time+1,2);
U_inf_2 = sqrt(Simulation.Global_Velocity(time+1,1)^2+Simulation.Global_Velocity(time+1,2)^2);
clc, clear Simulation

control__phi_ini = 0;
while control__phi_ini == 0
    
gyre1 = mass0*U_inf_1^2/R - abs(Fy_trim1);
gyre2 = mass0*U_inf_2^2/R - abs(Fy_trim2);
aux_trim1 = 1-abs(gyre1)/abs(gyre1 - gyre2); %1-abs(gyre1)/abs(mass0*U_inf_1^2/R);
aux_trim2 = 1-abs(gyre2)/abs(gyre1 - gyre2); %1-abs(gyre2)/abs(mass0*U_inf_2^2/R);
phi_ini_trim_med = (phi_ini_trim1*aux_trim1 + phi_ini_trim2*aux_trim2);

phi_ini = phi_ini_trim_med;
assignin('base','phi_ini',phi_ini)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Fy_trim_med = Simulation.Global_Forces(time+1,2);
U_inf_med = sqrt(Simulation.Global_Velocity(time+1,1)^2+Simulation.Global_Velocity(time+1,2)^2);
clc, clear Simulation

gyre = mass0*U_inf_med^2/R - abs(Fy_trim_med);
    if abs(gyre) < precision*100
        control__phi_ini = 1;
      elseif gyre < 0
        phi_ini_trim2 = phi_ini_trim_med;
        Fy_trim2 = Fy_trim_med;
      elseif gyre > 0
        phi_ini_trim1 = phi_ini_trim_med;
        Fy_trim1 = Fy_trim_med;
      end
    
end

phi_ini = phi_ini_trim_med
assignin('base','phi_ini',phi_ini)
clear aux_trim1 aux_trim2 U_inf_med U_inf_1 U_inf_2
clc

%%% Aileron Calculation

deltaa_trim1 = -1;
deltaa_trim2 = 1;

Aileron_Trim = deltaa_trim1;
assignin('base','Aileron_Trim',Aileron_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Mx_trim1 = Simulation.Body_Moments(time+1,1);
clc, clear Simulation

Aileron_Trim = deltaa_trim2;
assignin('base','Aileron_Trim',Aileron_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Mx_trim2 = Simulation.Body_Moments(time+1,1);
clc, clear Simulation

control__deltaa = 0;
while control__deltaa == 0
    
aux_trim1 = 1-abs(Mx_trim1)/abs(Mx_trim2-Mx_trim1);
aux_trim2 = 1-abs(Mx_trim2)/abs(Mx_trim2-Mx_trim1);
deltaa_trim_med = (deltaa_trim1*aux_trim1 + deltaa_trim2*aux_trim2);

Aileron_Trim = deltaa_trim_med;
assignin('base','Aileron_Trim',Aileron_Trim)
[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
Mx_trim_med = Simulation.Body_Moments(time+1,1);
clc, clear Simulation

    if abs(Mx_trim_med) < precision
        control__deltaa = 1;
      elseif Mx_trim_med*Mx_trim1<0
        deltaa_trim2 = deltaa_trim_med;
        Mx_trim2 = Mx_trim_med;
      elseif Mx_trim_med*Mx_trim2<0
        deltaa_trim1 = deltaa_trim_med;
        Mx_trim1 = Mx_trim_med;
      end

end

Aileron_Trim = deltaa_trim_med
assignin('base','Aileron_Trim',Aileron_Trim)
clear aux_trim1 aux_trim2
clc


[Simulation] = sim(Modelo, time_step*time);              % SIMULACION
%[u_ini, w_ini, Engine_Trim, Elevator_Trim]


U_inf_med = sqrt(Simulation.Global_Velocity(time+1,1)^2+Simulation.Global_Velocity(time+1,2)^2);
gyre = mass0*U_inf_med^2/R - abs(Simulation.Global_Forces(time+1,2));

if (abs(Simulation.Global_Forces(time+1,1)) < precision) & (abs(Simulation.Global_Forces(time+1,3)) < precision*500) & (abs(Simulation.Body_Moments(time+1,2)) < precision) & (abs(Simulation.Body_Moments(time+1,1)) < precision) & (abs(gyre) < precision*100)
  control__total = 1;
end

end

Aileron_Trim = -deltaa_trim_med/2
assignin('base','Aileron_Trim',Aileron_Trim)

%%
warning('on','all');
warning('query','all');
clc
toc
fprintf(['\n El giro de %.1f NM  de radio a %d ft se encuentra trimado para:\n u_ini =\t %g \t[m/s]\n w_ini =\t %g \t[m/s]\n Motor =\t %g \t[%c]\n Elevador =\t %g \t[º]\n Alerones =\t %g \t[º]\n\n'],R/1852,h*3.28,u_ini,w_ini,Engine_Trim*100,"%",Elevator_Trim*Max_Elevator*180/pi,Aileron_Trim*Max_Aileron*180/pi)






% set_param('Trim__SimulatorModel','FastRestart','off');
