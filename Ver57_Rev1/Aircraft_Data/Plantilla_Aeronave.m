% Datos de Aeronave
% Aeronave: Boeing 737-400
% Fuente: Dave Culp & Aeromatic (2006)
% Matlab Auth: Angel Daniel Velazquez Rodriguez (2022)

% Referencia adicional: https://www.varig-airlines.com/es/b737300.htm
% Referencia adicional: http://www.b737.org.uk/techspecsdetailed.htm

%clc, clear all, close all
Aircraft_Name = 'Boeing 737-300';

%% Propiedades Generales

mass0 = 83000 * 0.453592; %[kg] %52390; %[kg] - maximo al despegue %83000 * 0.453592; %[kg] - en vacío
cargo_mass = 0; %[kg]

fuel = 23170 * 0.2; %[l]
fuel_max = 23170; %[l]
fuel_density = 0.820 * 0.9899; %[kg/l] - Jet A a 25ºC

g = 9.80665; %[m/s^2]

Ixx = 562000 * 1.35581795; %[kg*m^2]
Iyy = 1.473e+06 * 1.35581795; %[kg*m^2]
Izz = 1.894e+06 * 1.35581795; %[kg*m^2]
Ixy = 0 * 1.35581795; %[kg*m^2]
Iyz = 0 * 1.35581795; %[kg*m^2]
Ixz = 8000 * 1.35581795; %[kg*m^2]

I = [Ixx Ixy Ixz
     Ixy Iyy Iyz
     Ixz Iyz Izz];

CGx = 639 * 0.0254; %[m]
CGy = 0 * 0.0254; %[m]
CGz = -40 * 0.0254; %[m]

% MODELO_3D = CurrentFolder + "\3DModels\" + "B737_MAX.ac"; MODELO_3D = char(MODELO_3D);
MODELO_3D = 'B737_MAX.ac';
    MODELO_3D_Data_Box = [-50000,50000,-50000,50000,-50000,50000];
    MODELO_3D_DataISO_Offset = [-1000,-1000,-1000];
    MODELO_3D_DataBack_Offset = [-1000,0,0];
    MODELO_3D_DataSide_Offset = [0,-1000,0];

SOUND_EFECT1 = CurrentFolder + "\Sounds\" + "SoundEfect_Sonido_de_Crucero_En_Avion_x3Vel.mp3"; SOUND_EFECT1 = char(SOUND_EFECT1);

OperativeCeiling = 12e3; %[m]

%% Propiedades de Propulsion
% CFM56
Engine_Name = 'CFM56';
T = 20000 * 4.4482; %[N] - x2 Motores
NMotores = 2;

dfuel0 = 661/3600 * 0.453592; %[kg/s]
dfuel = 1.41627252e-5; % 1.41627252e-5; % [kg/N/s] % 1.8504; %[kg/s]

% JT8D
% T = 14700 * 4.4482; %[N] - x2 Motores
% NMotores = 2;

Tpos_x = 540 * 0.0254; %[m]
Tpos_y = 193 * 0.0254; %[m]
Tpos_z = -40 * 0.0254; %[m]

KT_rho = [-10000   0   10000   20000   30000   40000   50000   60000];
KT_mach = [0.0 0.2 0.4 0.6 0.8 1.0 1.2];
KT = [  1.2600  1.0000  0.7400  0.5340  0.3720  0.2410  0.1490  0.0
        1.1710  0.9340  0.6970  0.5060  0.3550  0.2310  0.1430  0.0
        1.1500  0.9210  0.6920  0.5060  0.3570  0.2330  0.1450  0.0
        1.1810  0.9510  0.7210  0.5320  0.3780  0.2480  0.1540  0.0
        1.2580  1.0200  0.7820  0.5820  0.4170  0.2750  0.1700  0.0
        1.3690  1.1200  0.8710  0.6510  0.4750  0.3150  0.1950  0.0
        0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0]';

%% Atmosfera Asociada
temp0 = 288.15; % [K] - Temperatura
rho0 = 1.225; % [kg/m^3]
% h = 11000; % [m] - Altitud 
% temp = temp0 - (6.5e-3 *h); % [K] - Temperatura de Crucero
% rho = rho0* exp(-(g/(287*temp))*h) % [kg/m^3] 


%% Aerodinamica

S = 1171 * 0.092903; %[m^2]
c = 12.31 * 0.3048; %[m]
b = 94.70 * 0.3048; %[m]

Sh = 348 * 0.092903; %[m^2]
Sv = 297 * 0.092903; %[m^2]

lh = 48.04 * 0.3048; %[m]
lv = 44.50 * 0.3048; %[m]

% Lift (Eje Z) ------------------------------------------------------
CL_ajus_coef = 1;%0.86;%0.709675; % Techo alpha = 10

CL_alpha_reg = [-0.20   -0.68
                 0.00	0.20
                 0.23	1.20
                 0.46	0.20];
             
CL_alpha_interp = polyfit(CL_alpha_reg(:,1),CL_alpha_reg(:,2),3);


dCLflap = 0.9;

CLde = 0.2; % Sustentación debida al elevador
             
% Drag (Eje X) ------------------------------------------------------
CD_ajus_coef = 1;

CD0_reg =   [-1.57	1.5000
             -0.26	0.0420
              0.00	0.0210
              0.26	0.0420
              1.57	1.5000];
          
CD0_interp = polyfit(CD0_reg(:,1),CD0_reg(:,2),2);
CD0_interp = CD0_interp;% - [0 0 CD0_interp(3)];
          
CDi = 0.043;

CD_mach_reg = [ 0.00	0.0000
            0.79	0.0000
            1.10	0.0230
            1.80	0.0150];
        
% CD_mach_interp = spline(CD_mach_reg(:,1),CD_mach_reg(:,2));

CDflap = 0.059;
CDgear = 0.015;
CDsb = 0.02;
CDsp = 0.04;

CD_beta_reg = [-1.57	1.2300
             -0.26	0.0500
              0.00	0.0000
              0.26	0.0500
              1.57	1.2300];
          
CD_beta_interp = polyfit(CD_beta_reg(:,1),CD_beta_reg(:,2),2);
CD_beta_interp = CD_beta_interp - [0 0 CD_beta_interp(3)];

CDde = 0.059;

% Fuerza Lateral (Eje Y) --------------------------------------------
CYb = -1;


% Momento Cabeceo (Pitch) -------------------------------------------
Cm_alpha = -0.6;
Cm0 = -0.008;%-0.06;%-0.008;%-0.08;

Cmde_reg = [0.0	-1.20
      2.0	-0.30]; %Pitch moment debido al elevador en función del mach
Cmde_interp = polyfit(Cmde_reg(:,1),Cmde_reg(:,2),1);
Cmde_ajus_coef = 1;%0.02;

Cmq = -27.0;
Cmadot = -16.0;

% Momento Guiñada (Yaw) ----------------------------------------------
Cnb = 0.26 * 1;
Cnr = -0.35;
Cndr = -0.20;

% Momento Alabeo (Roll) ----------------------------------------------
Clb = -0.09; % Mz debido a beta
Clp = -0.4;
Clr = 0.09;
Clda_Reg = [0.0	0.100
            2.0	0.033];% Roll moment coeficent debido a los alerones en función del Mach
Clda_interp = polyfit(Clda_Reg(:,1),Clda_Reg(:,2),1);
Cldr = 0.01;

% Ground Effect
kCDge_reg = [ 0.0000	0.0480
            0.1000	0.5150
            0.1500	0.6290
            0.2000	0.7090
            0.3000	0.8150
            0.4000	0.8820
            0.5000	0.9280
            0.6000	0.9620
            0.7000	0.9880
            0.8000	1.0000];
    
kCDge_interp = polyfit(kCDge_reg(:,1)*b,kCDge_reg(:,2),5);
max_h_kCDge = (max(kCDge_reg(:,1))*b);
min_h_kCDge = (min(kCDge_reg(:,1))*b);

kCLge_reg = [ 0.0000	1.2030
            0.1000	1.1270
            0.1500	1.0900
            0.2000	1.0730
            0.3000	1.0460
            0.4000	1.0280
            0.5000	1.0190
            0.6000	1.0130
            0.7000	1.0080
            0.8000	1.0060
            0.9000	1.0030
            1.0000	1.0020
            1.1000	1.0000];

kCLge_interp = polyfit(kCLge_reg(:,1)*b,kCLge_reg(:,2),5);
max_h_kCLge = (max(kCLge_reg(:,1))*b);
min_h_kCLge = (min(kCLge_reg(:,1))*b);

% Speedbrakes
kCLsb = 0.85;

% Spoilers
kCLsp = 0.6;

% Mechanical Limiters
Max_Elevator = 0.3; % [rad]
Max_Aileron = 0.35; % [rad]
Max_Rudder = 0.35; % [rad]

%% Frenos
mu_d = 0.2;
mu_c = 0.35;

%% Asistencia en Vuelo

AoA_alpha_max = 25; % [deg]
AoA_beta_max = 15; % [deg]
AoA_phi_max = 80; % [deg]

Coef_Elevator_Alpha = 0.25;

Coef_Rudder_Beta = 0.25*4*0;
Coef_Aileron_Beta = 0.25*2*0;

Coef_Aileron_Phi = 0.05;

Coef_Aileron_Rudder = 1.25*0;
Coef_Rudder_Aileron = 0;













%%

% figure(1)
% plot(CL_alpha_reg(:,1)*180/pi,CL_alpha_reg(:,2),'b')
% hold on
% xq1 = min(CL_alpha_reg(:,1))-(5/180*pi):0.01:max(CL_alpha_reg(:,1))+(10/180*pi);
% % yq1 = makima(CL_alpha_reg(:,1),CL_alpha_reg(:,2),xq1);
% yq1 = polyval(CL_alpha_interp,xq1)
% plot(xq1*180/pi,yq1,'r-')
% 
% figure(1)
% plot(CD0_reg(:,1)*180/pi,CD0_reg(:,2),'b')
% hold on
% xq1 = min(CD0_reg(:,1))-(5/180*pi):0.01:max(CD0_reg(:,1))+(5/180*pi);
% % yq1 = spline(CD0_reg(:,1),CD0_reg(:,2),xq1);
% yq1 = polyval(CD0_interp,xq1)
% plot(xq1*180/pi,yq1,'r-')

% figure(3)
% plot(CD_mach_reg(:,1)*180/pi,CD_mach_reg(:,2),'b')
% hold on
% xq1 = min(CD_mach_reg(:,1)):0.01:max(CD_mach_reg(:,1));
% yq1 = interp1(CD_mach_reg(:,1),CD_mach_reg(:,2),xq1,'pchip');
% plot(xq1*180/pi,yq1,'r-')

% figure(4)
% plot(kCDge_reg(:,1) * b,kCDge_reg(:,2),'b')
% hold on
% xq1 = (min(kCDge_reg(:,1))*b):0.01:(max(kCDge_reg(:,1))*b);
% yq1 = polyval(kCDge_interp,xq1);
% plot(xq1,yq1,'r-')
% 
% figure(5)
% plot(kCLge_reg(:,1) * b,kCLge_reg(:,2),'b')
% hold on
% xq1 = (min(kCLge_reg(:,1))*b):0.01:(max(kCLge_reg(:,1))*b);
% yq1 = polyval(kCLge_interp,xq1);
% plot(xq1,yq1,'r-')

% auxx = polyfit(CD0_reg(:,1),CD0_reg(:,2),3)
% polyval(auxx, 10*pi/180)

% 
% figure(2)
% plot(CD_beta_reg(:,1)*180/pi,CD_beta_reg(:,2),'b')
% hold on
% xq1 = min(CD_beta_reg(:,1)):0.01:max(CD_beta_reg(:,1));
% % yq1 = spline(CD0_reg(:,1),CD0_reg(:,2),xq1);
% yq1 = polyval(CD_beta_interp,xq1)
% plot(xq1*180/pi,yq1,'r-')





