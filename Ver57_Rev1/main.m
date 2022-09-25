
% Matlab Auth: Angel Daniel Velazquez Rodriguez (2022)
% clear all, close all, clc

% Description: General scrip to execute the subscript for Simulation

CurrentFolder = pwd;
time_step = 0.200;

Mission_Data = [0,0,0,0,0;0,0,0,0,0;0,0,0,0,0];     
Mission = 0;

while 1
switch main_Menu

    case 1
    %% Aircraft Data:
    cd(CurrentFolder + "\Aircraft_Data");

    aux = (CurrentFolder + "\Aircraft_Data\*.m"); aux = char(aux);
    Aircrafts = dir(aux);
    Aircrafts = rmfield(Aircrafts, 'folder');
    Aircrafts = rmfield(Aircrafts, 'date');
    Aircrafts = rmfield(Aircrafts, 'bytes');
    Aircrafts = rmfield(Aircrafts, 'isdir');
    Aircrafts = rmfield(Aircrafts, 'datenum');

        clear aux;

        AircraftSelect = menu('Select Available Aircraft Data:', Aircrafts.name);
        copyfile(Aircrafts(AircraftSelect).name, 'AircraftData.m')
        AircraftData
        delete AircraftData.m

    clear AircraftSelect Aircrafts;

    cd(CurrentFolder);
    break

    case 2
    %% Aircraft 3D Model (Actual Simulink Callback)
    % 
    % cd(CurrentFolder + "\3DModels");
    % 
    %     copyfile(MODELO_3D, char(CurrentFolder + "\" + string(MODELO_3D)));
    %     
    % cd(CurrentFolder);
    break

    case 3
    %% Initial State
    InitialState
    break
    
    case 4
    %% Geolocation & Actual Time
    GeoDate_Clock = clock;

    GeoDate_Time = GeoDate_Clock(4) * 3600 + GeoDate_Clock(5) * 60 + GeoDate_Clock(6); % Second of the day
    GeoDate_Day = datenum(char(today("datetime")),'dd-mmm-yyyy')-datenum(char("01-January-" + GeoDate_Clock(1)'),'dd-mmm-yyyy'); % Day of the year (x of 366)
    GeoDate_ApIndex = [0, 2, 3, 4, 5, 6, 7, 9, 12, 15, 18, 22, 27, 32, 39, 48, 56, 67, 80, 94, 111, 132, 154, 179, 207, 236, 300, 400];
    GeoDate_Ap = GeoDate_ApIndex(randi([1,length(GeoDate_ApIndex)],1));
    
       Lat = [36 44 05.91];
       Long = [-06 03 53.33];
       Elv = 28;

    GeoDate_LatLongElv = [dms2degrees(Lat) dms2degrees(Long) Elv]; % [Latitud, Longitud, Elevación] - Localización del Aeropuerto de Jerez [36.744722 -6.06 28] - Runway 20 [dms2degrees([36 45 15.37]) dms2degrees([-06 03 19.46]) 28]
    break
    
    case 5
    %% Mision Trim
    close all
    cd(CurrentFolder + "\Trim");
    
%             clear i
        for i = 1:length(aux)
                assignin('caller',char(aux(i)),evalin('base',char(aux(i))))    
        end
            clear i

    Trim_Menu = menu('Select the trim function:',...
                        'Horizontal Cruise Flight (theta)',...
                        'Steady Ascent/Descent (theta,Vertical Speed)',...
                        'Horizontal Turn (theta, Radius, Direction)',...
                        'Takeoff Condition',...
                        'Exit/No Trim');

    while 1
    switch Trim_Menu
        case 1 % Vuelo en Crucero Horizontal (theta)
            %load_system("Trim__SimulatorModel.mdl")
            
            phi_ini_aux = phi_ini; phi_ini = 0;
            psi_ini_aux = psi_ini; psi_ini = 0;
            
            run("Trim_CruiseThetaXZ2.m")

            phi_ini = phi_ini_aux; clear phi_ini_aux;
            psi_ini = psi_ini_aux; clear psi_ini_aux;
            
            break

        case 2 % Ascenso/Descenso Constante (theta,Vertical Speed)
            phi_ini_aux = phi_ini; phi_ini = 0;
            psi_ini_aux = psi_ini; psi_ini = 0;
            
                prompt = {'Enter the ascent/descent speed in [ft/min]'};
                dlgtitle = 'Ascent/Descent Speed';
                definput = {'400'};
                dims = [1 60];
                opts.Interpreter = 'tex';
                answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
                answer = str2double(cell2mat(answer));
            climb_rate_trim = answer*0.3048/60; % [m/s]
            clear prompt dlgtitle definput dims opts.Interpreter answer

            run("Trim_ClimbXZ2.m")

            phi_ini = phi_ini_aux; clear phi_ini_aux;
            psi_ini = psi_ini_aux; clear psi_ini_aux;
            break

        case 3 % Viraje Horizontal (theta, Radius, Direction)
            psi_ini_aux = psi_ini; psi_ini = 0;
            phi_ini = 0;
            
                prompt = {'Enter the radius of gyration in [m]','Enter the direction: Left -> -1 | Right -> 1'};
                dlgtitle = 'Turning Conditions';
                definput = {'20000','1'};
                dims = [1 60;1 60];
                opts.Interpreter = 'tex';
                answer = inputdlg(prompt,dlgtitle,dims,definput,opts);
                answer = [str2double(cell2mat(answer(1))), str2double(cell2mat(answer(2)))];
                if abs(answer(2)) ~= 1
                    answer(2) = 1;
                end
            R = answer(1); % [m]
            Direc = answer(2); % Left -> -1 | Right -> 1
            clear prompt dlgtitle definput dims opts.Interpreter answer

            run("Trim_ArcDME2.m")

            psi_ini = psi_ini_aux; clear psi_ini_aux;
            clear R Direct;
            break

        case 4 % Condicion de Despegue
            u_ini = 0;
            v_ini = 0;
            w_ini = 0;
            phi_ini = 0;
            theta_ini = 0.000;
            psi_ini = (23-90-0.7)*pi/180; % RunWay Direction
            x_ini = 0;
            y_ini = 0;
            z_ini = 0;
            Engine_Trim = 0.0;
            Elevator_Trim = 0.0;% 0.057;
            break

        case 5 % EXIT
            break
    end
    end
    clear Trim_Menu
    delete *.slxc
    delete *.asv
    if exist(CurrentFolder + "\Trim\slprj")
    rmdir(CurrentFolder + "\Trim\slprj", 's')
    end
    delete(CurrentFolder + "\*.ac")
    break

    case 6 
    %% Joystick Verification
        JoyStick_Connection = NaN;
        JoyStick_Connection = axis(vrjoystick(1),1);
    break
    
    case 7
        close all
        Modelo = 'Mission__SimulatorModel';
        
        cd(CurrentFolder + "\Trim");
            for i = 1:length(aux)
                    assignin('caller',char(aux(i)),evalin('base',char(aux(i))))    
            end
                clear i
                
        [Simulation] = sim(Modelo, 10000);
        close_system(Modelo);
            delete *.slxc
            delete *.asv
            if exist(CurrentFolder + "\Trim\slprj")
            rmdir(CurrentFolder + "\Trim\slprj", 's')
            end
            clc
    break
end
end
cd(CurrentFolder);

    VarsNames = who;

