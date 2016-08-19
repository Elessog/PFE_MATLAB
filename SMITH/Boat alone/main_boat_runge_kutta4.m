close all;clc;
global index_out q max_windspeed time_const_wind v_target psi ...
    controller_freq size_rect_cont control_computed delay buffer_command...
    command_buffer_size delta_r delta_s idx_bfc...
    delta_r_s delta_s_s pos_sum active_os tacking;

%%%%%% Time parameters %%%%%%%
stepH = 0.01;
x= 0:stepH:600;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

controller_freq = 2;
size_rect_cont = 0.1;
control_computed = 0;
delay = 2;
command_buffer_size =14; %floor(controller_freq*delay)+10;
buffer_command = zeros(3,command_buffer_size);
idx_bfc = 1;
delta_r = 0;
delta_s = 0;
delta_r_s = 0;
delta_s_s = 0;
length_delay = floor(delay/stepH)+1;
active_os = 0;
tacking = 0;
%% initialization of the state of the boat
origin = [0;0];

index_out=1;%#ok<NASGU> %index for the path planning script
q=1; %tacking variable
max_windspeed = 3;
time_const_wind = 10;
v_target = 2.3;
theta_0 = 0*pi/4;
psi=-3*0*pi/4;

%% creation vector y for the differential solving

y0 = [origin;theta_0;0;0];

y = zeros(length(y0),length(x));
y_s = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
y_s(:,1) = y0;  
F_xy =@(t,y) boat_simulation(t,y);                    % change the function as you desire
Fs_xy =@(t,y) boat_simulation_smith(t,y);                    % change the function as you desire
sauv_delta = zeros(4,length(x));
pos_sum = [0,0,0];
pos_to_controller = zeros(length(x),3);
%% computation of the ODE
for i=1:length(x)-1                            % calculation loop
    
    if (i>length_delay && i > 1)
        %active otto_smith
        active_os = 1;
        pos_sum_t = y_s(1:3,i-1) -...
            y_s(1:3,i-length_delay);
        pos_sum = pos_sum_t';
    end
    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_2);
    k_4 = F_xy(x(i)+stepH,y(:,i)+k_3*stepH);
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
    k_1 = Fs_xy(x(i),y_s(:,i));
    k_2 = Fs_xy(x(i)+0.5*stepH,y_s(:,i)+0.5*stepH*k_1);
    k_3 = Fs_xy(x(i)+0.5*stepH,y_s(:,i)+0.5*stepH*k_2);
    k_4= Fs_xy(x(i)+stepH,y_s(:,i)+k_3*stepH);
    y_s(:,i+1) = y_s(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation of predictive
    %% sauvegarde
    sauv_delta(:,i) = [delta_r;delta_s;delta_r_s;delta_s_s];
    pos_to_controller(i,:) = y(1:3,i)'+pos_sum;
end
y= y';


%% data reorganization

pos_boat = y(:,1:2);
theta_boat = y(:,3);
v =  y(:,4);
theta_dot_boat=  y(:,5);

%% Instruction
% to see data save the workspace as a .mat and launh plot multi

%% visu
% 
% 
% index_out=1; %restarting for the controller for visualisation
% 
% p1 = 0.05;
% a2 = 2;
% figure(666)
% jk = 0;
% for i=1:10:length(x)-1
%     %% boat
%     %figure(668)
%     clf
%     pos_b = pos_boat(i,:);
%     [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
%     [delta_r, delta_s] = controller_simpleLine_v_control(pos_b(1) ,pos_b(2), theta_boat(i),v(i) ,psi, a, b2);
% 
%     %clf         %clear current figure
%     hold on
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis square
%     axis_max_l = 100;
%     axis_min = -2;
%     s = axis_max_l*.04;
%     axis([axis_min-s axis_min+axis_max_l+s axis_min-s axis_min+axis_max_l+s]);
%     
%     
%     %draw wind direction
%     m_x = axis_min+axis_max_l/2;
%     m_y = axis_min+axis_max_l/2;
%     x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
%     y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
%     line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
%     line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
%     line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
%     line([a(1) b2(1)],[a(2) b2(2)],'color','black');
%       
%     sing = sign(sin(theta_boat(i)-psi));
%     if sing==0
%         sing = 1;
%     end
%     draw_boat([],s,pos_b(1),pos_b(2),theta_boat(i),delta_r,sing*delta_s);
%     %line([x,x+0.05*axis_max_l*cos(alpha_cable)],[y,y+0.05*axis_max_l*sin(alpha_cable)],'color','c')
%     %delta_r
%     %delta_sMax
%     title_f = sprintf('Time : %0.3f s theta %0.2f',i*stepH,theta_boat(i));
%     title(title_f);
% 
%     pause(stepH*1)
%     
% end
