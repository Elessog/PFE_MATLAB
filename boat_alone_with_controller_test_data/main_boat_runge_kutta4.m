clear all;close all;clc;


[FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');

global waypoints;
load(FileName);

labview_waypoints = 0;
if labview_waypoints
    [FileName2,PathName2] = uigetfile('*.mat','Select the labview waypoints');
    load(FileName2);
    waypoints = [utm_x-origin(1) utm_y-origin(2) (1:length(utm_x))'];
end

origin2=origin;
waypoints_t =waypoints;
waypoints(:,3) = 10*ones(length(waypoints(:,1)),1);

global index_out q psi windspeed_t...
    controller_freq size_rect_cont control_computed delay buffer_command...
    command_buffer_size delta_r delta_s idx_bfc...
    delta_r_s delta_s_s pos_sum active_os i_way;

i_way=2;
%% preprocessing of test data
heading_comp = heading.*(v>=1)+heading2.*(v<1);
time = fixtime(time);
size_buff = 2;
v_real = v;
v_t = v_real;
v_real = zeros(1,length(v));
for i=1:length(v)
    v_real(i) = mean(v_t(max(1,i-size_buff):min(i+size_buff,length(v))));
end


accel = diff(v_real)./diff(time);
old_v = v;

accel = [accel,0];
delta_r_ar =(arduino(2,:)-285)*1600/235*(pi/6)/1500;

app_old = -(tw_d-pi-pi/2)-(-(heading2-pi/2));
twSpeed = sqrt((windspeed.^2+old_v.^2-(2*old_v.*windspeed.*cos(app_old))));
alpha = acos((windspeed.*cos(app_old)-old_v)./twSpeed);
windspeed= twSpeed;
tw_d = mod(pi/2-alpha+pi,2*pi);

%%

%%%%%% Time parameters %%%%%%%
stepH = 0.01;
x= time(1):stepH:time(end-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


controller_freq = 2;
size_rect_cont = 0.1;
control_computed = 0;
delay = 1;
command_buffer_size =14; %floor(controller_freq*delay)+10;
buffer_command = zeros(3,command_buffer_size);
idx_bfc = 1;
delta_r = 0;
delta_s = 0;
delta_r_s = 0;
delta_s_s = 0;
length_delay = floor(delay/stepH)+1;
active_os = 0;
%% initialization of the state of the boat
origin = [east_north(1,1);east_north(2,1)];

index_out=1;%#ok<NASGU> %index for the path planning script
q=1; %tacking variable

theta_0 = heading_comp(1);
windspeed_t = windspeed(1);
psi=tw_d(1);%+pi;

delta_r = delta(1,1);
delta_s = delta(2,1);
delta_r_s = delta(1,1);
delta_s_s = delta(2,1);

%% creation vector y for the differential solving

y0 = [origin;theta_0;v(1);0];


y = zeros(length(y0),length(x));
y_s = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
y_s(:,1) = y0;  
F_xy =@(t,y) boat_simulation(t,y);                    % change the function as you desire
sauv_delta = zeros(4,length(x));
wind_save = zeros(length(x),2);
pos_sum = [0,0];
pos_to_controller = zeros(length(x),2);

%% computation of the ODE
for i=1:length(x)-1                            % calculation loop
    
    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_2);
    k_4 = F_xy(x(i)+stepH,y(:,i)+k_3*stepH);
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
    %% delta calc
    time_idx = find(time>x(i+1),1,'first');
    if(mod(i,1000)==0)
        fprintf('Time: %.1f/%d  index: %d  %d\n',time(time_idx)-time(1),time(end-1)-time(1),time_idx,i);
    end
    dt_temp = (time(time_idx)-x(i+1))/(time(time_idx)-time(time_idx-1));
    %delta_r =delta_r_ar(time_idx)-dt_temp*(delta_r_ar(time_idx)-delta_r_ar(time_idx-1));
    
    windspeed_t = windspeed(time_idx);
    psi=tw_d(time_idx);
    sauv_delta(:,i) = [delta_r;delta_s;delta_r_s;delta_s_s];
    %delta_r = delta_r_ar(1,time_idx);
    %delta_s = delta(2,time_idx);
    wind_save(i,:) = [windspeed_t,psi ];
end
y= y';


%% data reorganization

pos_boat = y(:,1:2);
theta_boat = y(:,3);
v =  y(:,4);
theta_dot_boat=  y(:,5);

%% visu

timeJump = 100;
omega_dot_v = zeros(length(1:timeJump:length(x)-1),3);
v_down = zeros(length(1:timeJump:length(x)-1),1);
v_dot_= zeros(length(1:timeJump:length(x)-1),2);
f_rudder_= zeros(length(1:timeJump:length(x)-1),2);
f_sail_= zeros(length(1:timeJump:length(x)-1),2);
f_frict_=  zeros(length(1:timeJump:length(x)-1),2);

index_out=1; %restarting for the controller for visualisation



min_x = min(pos_boat(:,1));
max_x = max(pos_boat(:,1));
min_y = min(pos_boat(:,2));
max_y = max(pos_boat(:,2));

use_waypoint=0;

if use_waypoint
    min_x = min(waypoints(:,1));
    max_x = max(waypoints(:,1));
    min_y = min(waypoints(:,2));
    max_y = max(waypoints(:,2));
end

lar = max([max_y-min_y,max_x-min_x]);

p1 = 0.05;
a2 = 2;
figure(666)
jk = 0;
print = 1;
for i=1:timeJump:length(x)-1
    %% boat
    %figure(668)
    if print
        clf
    end
    jk = jk+1;
    pos_b = pos_boat(i,:);
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [v_dot_main,omega_dot_t,f_rudder,f_sail,f_frict] = model_sailboat_jaulin_modified4_visu(y(i,:),wind_save(i,1),wind_save(i,2),sauv_delta(4,i),sauv_delta(3,i));
    omega_dot_v(jk,:) = omega_dot_t;
    v_down(jk) =v(i);
    v_dot_(jk,:) = v_dot_main;
    f_rudder_(jk,:) = f_rudder;
    f_sail_(jk,:) = f_sail;
    f_frict_(jk,:) = f_frict;
    %clf         %clear current figure
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    axis square
    axis_max_l = lar;
    s = axis_max_l*.04;
    
    axis_min = -20;
    axis([min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min]);
    
    %draw wind direction
    m_x = min_x+axis_min+axis_max_l/2;
    m_y = min_y+axis_min+axis_max_l/2;
    x_w = [m_x m_x+3*s*cos(wind_save(i,2)) m_x+3*s*cos(wind_save(i,2))-s*cos(wind_save(i,2)-pi/4) m_x+3*s*cos(wind_save(i,2))-s*cos(wind_save(i,2)+pi/4)];
    y_w = [m_y m_y+3*s*sin(wind_save(i,2)) m_y+3*s*sin(wind_save(i,2))-s*sin(wind_save(i,2)-pi/4) m_y+3*s*sin(wind_save(i,2))-s*sin(wind_save(i,2)+pi/4)];
    
    if print
        line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
        line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
        line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
        
        %link equations
        W_ap = [wind_save(i,1)*cos(wind_save(i,2)-y(i,3))-y(i,4) wind_save(i,1)*sin(wind_save(i,2)-y(i,3))];
        %apperent wind speed vector in b-frame
        phi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame
        a_ap = hypot(W_ap(1),W_ap(2));      %apperent wind speed velocity in b-frame

        sigma=cos(phi_ap)+cos(sauv_delta(4,i));
        if (sigma<0),
            delta_s_show=pi+phi_ap;
        else
            if sin(phi_ap)~=0
                delta_s_show=-sign(sin(phi_ap))*abs(sauv_delta(4,i));
            else
                delta_s_show = sauv_delta(4,i);
            end
        end;
        draw_boat([],s,pos_b(1),pos_b(2),theta_boat(i),sauv_delta(3,i),delta_s_show);
        viscircles(waypoints(:,1:2),waypoints_t(:,3));
        %line([x,x+0.05*axis_max_l*cos(alpha_cable)],[y,y+0.05*axis_max_l*sin(alpha_cable)],'color','c')
        %delta_r
        %delta_sMax
        title_f = sprintf('Time : %0.3f s',i*stepH);
        title(title_f);
        
        pause(stepH*10)
    end
end


%%
figure
plot(x(1:timeJump:length(x)-1),[v_dot_ f_rudder_ f_sail_ f_frict_])

h=legend('$v_{\dot{x}}$','$v_{\dot{y}}$','$f_{rudderX}$','$f_{rudderY}$','$f_{sailX}$','$f_{sailY}$','$f_{frictX}$','$f_{frictY}$');
set(h,'Interpreter','latex')



theta_c = mod(theta_boat(1:timeJump:length(x)-1),2*pi);
figure
v_dot_r = v_dot_(:,1)./cos(theta_c);
f_rudder_r= f_rudder_(:,1)./cos(theta_c);
f_sail_r= f_sail_(:,1)./cos(theta_c);
f_frict_r= f_frict_(:,1)./cos(theta_c);

subplot(1,2,1)

plot(x(1:timeJump:length(x)-1),[v_dot_r f_rudder_r f_sail_r f_frict_r])
%axis([time(1) 235 -2 2])
h=legend('$v_{dot}$','$f_{rudder}$','$f_{sail}$','$f_{frict}$');
set(h,'Interpreter','latex')
subplot(1,2,2)
plot(time(1:end),accel);
%axis([time(1) 235 -0.02 0.02])

figure
subplot(1,2,1)
plot(time,old_v)
axis([time(1) 250 1 2])
title('Real speed')

subplot(1,2,2)
plot(x,v)
axis([time(1) 250 1 2])
title('Simu speed')

%%
figure
hold on
plot(east_north(1,:),east_north(2,:),'r')
plot(pos_boat(:,1),pos_boat(:,2))
viscircles(waypoints(:,1:2),waypoints(:,3));




%% simulation to
t_xr= 1:timeJump:length(x)-1;
x_r = x(t_xr);
[pos_lat,pos_lont] = utm2ll(pos_boat(t_xr,1)'+origin2(1),pos_boat(t_xr,2)'+origin2(2),34);

time = fixtime(time);
description ='postion of boat';
name = 'Simu_without_cable';
filename = ['simu_',FileName(1:end-3),'kml'];
kmlStr = ge_track(t_xr*stepH/24/3600,pos_lat,pos_lont,zeros(1,length(x_r)),...
    'name',name,...
    'lineColor','#FF22FFFF',...
    'lineWidth',5,...
    'extendedData',{'Speed',v(1:timeJump:length(x)-1);'Rudder_Act', sauv_delta(1,1:timeJump:length(x)-1)});
ge_output(filename,kmlStr,'name',name)

