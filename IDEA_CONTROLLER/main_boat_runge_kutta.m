close all;clc;
global index_out q max_windspeed time_const_wind v_target psi ...
    controller_freq size_rect_cont control_computed delay buffer_command...
    command_buffer_size delta_r delta_s idx_bfc...
    delta_r_s delta_s_s pos_sum active_os ...
    v_dot fs phi_ap W_ap coeff_rudder_speed;
global old_t old_diff_v old_diff_v_dot coeff_d_rudder_speed diff_v_dot tacking;
global error_diff_v coeff_anti_windup boat_v;

%%%%%% Time parameters %%%%%%%
stepH = 0.01;
x= 0:stepH:2000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




controller_freq = 8;
size_rect_cont = 0.1;
control_computed = 0;
delay = 0;
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
fs=0;
v_dot=0;
phi_ap=0;
W_ap = [0,0];
boat_v = 0;
%% initialization of the state of the boat

coeff_div_pressure_sensor = 2;
length_cable = 20;
boolPrint = 1;
depth_target = -6.3;


press_norm_0 =-5.3;

tacking=0;

old_t=0;
old_diff_v=0;
old_diff_v_dot=0;
diff_v_dot = 0;
coeff_d_rudder_speed = -1;
coeff_rudder_speed = -1/20;
coeff_anti_windup = 1;
error_diff_v = 0;

origin = [0;0;-0.3];

index_out=1;%#ok<NASGU> %index for the path planning script
q=1; %tacking variable
max_windspeed = 3;
time_const_wind = 0.1;
theta_0 = 0*pi/4;
psi=-3*0*pi/4;

%% creation vector y for the differential solving

y0 = [origin;theta_0;0;0;0];

y = zeros(length(y0),length(x));
y_s = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
y_s(:,1) = y0;  
F_xy =@(t,y) boat_simulation_aw(t,y);                           % change the function as you desire
sauv_fs_vdot = zeros(2,length(x));
sauv_delta = zeros(2,length(x));
sauv_phi_ap = zeros(1,length(x));
sauv_w_ap = zeros(2,length(x));
pos_sum = [0,0,0];
%% computation of the ODE
for i=1:length(x)-1                            % calculation loop
    

    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_2);
    k_4 = F_xy(x(i)+stepH,y(:,i)+k_3*stepH);
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
    sauv_fs_vdot(:,i+1) = [v_dot fs];
    sauv_delta(:,i+1) = [delta_r,delta_s];
    sauv_phi_ap(:,i+1) = phi_ap;
    sauv_w_ap(:,i+1) = W_ap';
end
y= y';


%% data reorganization

pos_boat = y(:,1:2);
theta_boat = y(:,3);
v =  y(:,4);
theta_dot_boat=  y(:,5);

%% visu

%v = VideoWriter('newfile.avi','Uncompressed AVI');
%aviobj = avifile('example_osci.avi','compression','None','fps',25);

draw_cable_ = 0;

ratio = 1;
v_2 = zeros(length(1:ratio:length(x)-1),1);

v_ = v_2;
diff_v_ = diff(pos_boat)/stepH;

v_(1:end) = sqrt(diff_v_(:,1).^2+diff_v_(:,2).^2);

if draw_cable_
    figure(666)
end

jk = 0;
for i=1:ratio:length(x)-1
    %% cable
    jk = jk+1;
    
    time_idx = find(time>=x(i),1,'first');
    
    time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
    v_2(jk)=v_(i);
    
    pos_b = pos_boat(i,:);
    l = sum(L);
    
    l = pos_boat(i,:);
      
    
    
    if draw_cable_
        clf
        
        axis([-10+pos_b(1) 10+pos_b(1)...
            -10+pos_b(2) 10+pos_b(2)...
            -10+pos_b(3) 2+pos_b(3)])
        axis vis3d
        title(sprintf('Time : %.3f',i*stepH));
        drawnow
        %aviobj = addframe(aviobj,gcf);
        pause(ratio*stepH/1)
        
    end
end

%%
%viobj = close(aviobj)

%% draw position
pointx = [0,20,40,60,30,0]*10; %,40,60,30,0
pointy = [0,0,20,10,0,0]*10;%,20,10,0,0

figure
hold on
plot(pos_boat(:,1) ,pos_boat(:,2),'r')
plot(pointx,pointy,'--')
hold off
title('Path of the boat')
legend('Path of the boat','Line to follow')

%% analyse
rho = 1000;
radius=0.005;
ms=sum(m);
g=9.81;
CD=1.2;

time_2 = x(1:ratio:length(x)-1);
v_pos = diff(pos_boat)/stepH;
v_2_simu =sqrt(v_pos(:,1).^2+v_pos(:,2).^2);
L_=length_cable;
depth_comp_2 =-cos(atan(CD*2*radius*L_*rho*v_2_simu.^2/(2*g*(rho*pi*L_*radius^2-ms))))*L_-0.3; 

figure
subplot(2,1,1)
hold on
plot(time_2-time_2(1),depth_comp_2(:),'r');
plot(time_2-time_2(1),depth_target*ones(size(time_2)),'g--');
hold off
t=title(['Simulation Depth cable over time ']);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
legend('Depth of cable','target')
subplot(2,1,2)
plot(time_2-time_2(1),v_2)
xlabel('Time (s)')
ylabel('Speed (m/s)')
