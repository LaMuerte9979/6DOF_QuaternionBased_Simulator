% clear all, close all, clc

% Auth: Angel Daniel Velazquez Rodriguez (2022)

% Boeing737_Data

% InitialState

% Trim_CruiseXZ
% clear Modelo

Modelo = 'Trim__SimulatorModel'

%% Trimmed Data
tic
precision = 1;
control__total = 0;

warning('off','all');
warning

% climb_rate_trim = 400*0.3048/60; % [m/s]
% climb_angle_trim = 1*pi/180; % [rad]

% Simulation.Global_Velocity


while control__total == 0;
    
%%% Body Velocity Calculation 1
control__angle = 0;
while control__angle == 0;
% quat = quaternion([-phi_ini,-theta_ini,-psi_ini],'rotvecd'); %version en [deg]
quat = quaternion([phi_ini,theta_ini,psi_ini],'rotvec'); %version en [rad]
quat = normalize(quat);
quat_vect = compact(quat);

UVW_trim = [320,0,0];

uvw_trim = quatrotate(quat_vect,UVW_trim);

u_ini_trim1 = 0;
u_ini_trim2 = uvw_trim(1);

v_ini_trim1 = 0;
v_ini_trim2 = uvw_trim(2);

w_ini_trim1 = 0;
w_ini_trim2 = uvw_trim(3);

clear quat quat_vect uvw_trim2 UVW_trim


u_ini = u_ini_trim1;
assignin('base','u_ini',u_ini)
v_ini = v_ini_trim1;
assignin('base','v_ini',v_ini)
w_ini = w_ini_trim1;
assignin('base','w_ini',w_ini)
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fz_trim1 = Simulation.Global_Forces(3);
clc, clear Simulation

u_ini = u_ini_trim2;
assignin('base','u_ini',u_ini)
v_ini = v_ini_trim2;
assignin('base','v_ini',v_ini)
w_ini = w_ini_trim2;
assignin('base','w_ini',w_ini)
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fz_trim2 = Simulation.Global_Forces(3);
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
UVW_aux1 = quatrotate(quat_vect,[0 0 climb_rate_trim]);
UVW_aux2 = quatrotate(quat_vect,[u_ini_trim_med 0 0]);


u_ini = u_ini_trim_med;% + UVW_aux1(1);
assignin('base','u_ini',u_ini)
% w_ini = -w_ini_trim_med;
w_ini = -UVW_aux2(3) + UVW_aux1(3);
assignin('base','w_ini',w_ini)
clear UVW_aux
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fz_trim_med = Simulation.Global_Forces(3);
Vz_trim = Simulation.Global_Velocity(3);
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

%     if abs(Vz_trim-climb_rate_trim) < precision
        control__angle = 1;
%         elseif Vz_trim > climb_rate_trim
%             theta_ini = theta_ini - precision/180*pi;% * (Vz_trim-climb_rate_trim)/climb_rate_trim;
%         elseif Vz_trim < climb_rate_trim
%             theta_ini = theta_ini + precision/180*pi;% * (Vz_trim-climb_rate_trim)/climb_rate_trim;
%         end
end
quat = quaternion([-phi_ini,theta_ini,psi_ini],'rotvec');
quat = normalize(quat);
quat_vect = compact(quat);
UVW_aux1 = quatrotate(quat_vect,[0 0 climb_rate_trim]);
UVW_aux2 = quatrotate(quat_vect,[u_ini_trim_med 0 0]);

u_ini = u_ini_trim_med;% + UVW_aux1(1);
assignin('base','u_ini',u_ini)
w_ini = -UVW_aux2(3) + UVW_aux1(3);
assignin('base','w_ini',w_ini)
% w_ini = -w_ini_trim_med

clear aux_trim1 aux_trim2 UVW_aux;
clear quat quat_vect;

%%% Engine Lever Calculation

lever_trim1 = 0;
lever_trim2 = 1;

Engine_Trim = lever_trim1;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fx_trim1 = Simulation.Global_Forces(1);
clc, clear Simulation

Engine_Trim = lever_trim2;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fx_trim2 = Simulation.Global_Forces(1);
clc, clear Simulation

control__engine = 0
while control__engine == 0
aux_trim1 = 1-abs(Fx_trim1)/abs(Fx_trim2-Fx_trim1);
aux_trim2 = 1-abs(Fx_trim2)/abs(Fx_trim2-Fx_trim1);
lever_trim_med = (lever_trim1*aux_trim1 + lever_trim2*aux_trim2);

Engine_Trim = lever_trim_med;
assignin('base','Engine_Trim',Engine_Trim)
[Simulation] = sim(Modelo, 0);              % SIMULACION
Fx_trim_med = Simulation.Global_Forces(1);
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
[Simulation] = sim(Modelo, 0);              % SIMULACION
My_trim1 = Simulation.Body_Moments(2);
clc, clear Simulation

Elevator_Trim = deltae_trim2;
assignin('base','Elevator_Trim',Elevator_Trim)
[Simulation] = sim(Modelo, 0);              % SIMULACION
My_trim2 = Simulation.Body_Moments(2);
clc, clear Simulation

control__deltae = 0;
while control__deltae == 0
    
aux_trim1 = 1-abs(My_trim1)/abs(My_trim2-My_trim1);
aux_trim2 = 1-abs(My_trim2)/abs(My_trim2-My_trim1);
deltae_trim_med = (deltae_trim1*aux_trim1 + deltae_trim2*aux_trim2);

Elevator_Trim = deltae_trim_med;
assignin('base','Elevator_Trim',Elevator_Trim)
[Simulation] = sim(Modelo, 0);              % SIMULACION
My_trim_med = Simulation.Body_Moments(2);
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

[Simulation] = sim(Modelo, 0);              % SIMULACION
%[u_ini, w_ini, Engine_Trim, Elevator_Trim]


if (abs(Simulation.Global_Forces(1)) < precision) & (abs(Simulation.Global_Forces(3)) < precision) & (abs(Simulation.Body_Moments(2)) < precision)
  control__total = 1;
end

end


%%
warning('on','all');
warning('query','all');
clc
toc
fprintf(['\n El cambio de nivel a %g ft/min desde los %d ft se encuentra trimado para:\n theta_ini =\t %g \t[º]\n u_ini =\t %g \t[m/s]\n w_ini =\t %g \t[m/s]\n Motor =\t %g \t[%c]\n Elevador =\t %g \t[º]\n\n'],climb_rate_trim/0.3048*60,h*3.28,theta_ini*180/pi,u_ini,w_ini,Engine_Trim*100,"%",Elevator_Trim*Max_Elevator*180/pi)
