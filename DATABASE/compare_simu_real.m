%plot
close all;
if ~exist('run_charged','var')
    run_charged = 0;
else
    choice =questdlg('Would you want to restart simulations?','Yes','No');
    switch choice
        case 'Yes'
            run_charged = 0;
    end
end

if ~run_charged
    clear all
    [FileName1,PathName] = uigetfile('*.mat','Select the real run');
    
    load([PathName,FileName1]);

    waypoints_1 = waypoints;
    time_1 = time;
	east_north_1 = east_north;
	heading_1 = heading;
	heading2_1= heading2;
	yaw_2_1 = yaw_2;
	yaw_1 = yaw;
	v_1 = v;
    windspeed_1 = windspeed;
    tw_d_1 = tw_d;
    arduino_1 =arduino;
    press1 = -3*(arduino_1(1,:)-181)/(378-181);
    
    [FileName3,PathName3] = uigetfile('*.mat','Select the simulation run');
    
    load([PathName3,FileName3]);


    waypoints_2 = waypoints;
    time_2_simu = time_2;
    time_2 = time;
	east_north_2 = east_north;
	heading_2 = heading;
	heading2_2= heading2;
	yaw_2_2 = yaw_2;
	yaw_2 = yaw;
    v_2_simu = v_2;
	v_2 = v;
    windspeed_2 = windspeed;
    tw_d_2 = tw_d;
    arduino_2 =arduino;
    press2 = -3*(arduino_2(1,:)-181)/(378-181);
    
    run_charged = 1;
end



%% Depth

figure
hold on
plot(time_1-time_1(1),(press1+0.3)*sum(L)/3-0.3,'g')
plot(time_2_simu-time_2_simu(1),rod_end_n,'r--')
t=title(['Depth cable over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
legend('Real Test','Simulation')


figure
hold on
plot(time_1-time_1(1),press1,'g')
plot(time_2_simu-time_2_simu(1),rod_end_n_2,'r--')
t=title(['Depth cable at the pressure sensor level over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
legend('Real Test','Simulation')



rho = 1000;
radius=0.005;
ms=sum(m);
g=9.81;
CD=1.2;
L_=6;


v_compute = min(v_1):0.1:max(v_1);
figure
hold on 
sparsing = 1;
depth_comp = -cos(atan(CD*2*radius*L_*rho*v_compute/(2*g*(rho*pi*L_*radius^2-ms))))*L_-0.3;

angle_ = atan(CD*2*radius*rho*L_*v_1(1:sparsing:end)/(2*(rho*pi*L_*radius^2-ms)));

% figure
% plot(v_1,angle_);

plot(v_1(1:sparsing:end),(press1(1:sparsing:end)+0.3)*sum(L)/3-0.3,'xg')
plot(v_2_simu(1:sparsing:end),rod_end_n(1:sparsing:end),'+r')
plot(v_compute,depth_comp,'b');
t=title(['Depth cable over speed ',FileName]);
set(t,'Interpreter','None')
xlabel('Speed (m/s)')
ylabel('Depth (m)')
legend('Real Test','Simulation','single pendulem computed')



figure
hold on 
sparsing = 5;
plot(v_1(1:sparsing:end),press1(1:sparsing:end),'xg')
plot(v_2_simu(1:sparsing:end),rod_end_n_2(1:sparsing:end),'+r')
t=title(['Depth cable  at the pressure sensor level over speed ',FileName]);
set(t,'Interpreter','None')
xlabel('Speed (m/s)')
ylabel('Depth (m)')
legend('Real Test','Simulation')





%%

size_waypoints = length(waypoints_1(:,1));
waypoint_enter_exit_time_1 = [];
waypoint_enter_exit_time_2 = [];

reach = 15;

deb_waypoints = 2;

for i = deb_waypoints:size_waypoints
      res = (east_north_1(2,:)-waypoints_1(i,2)).^2+(east_north_1(1,:)-waypoints_1(i,1)).^2 <  reach.^2; % is there point close to waypoint
      res2 = (east_north_2(2,:)-waypoints_2(i,2)).^2+(east_north_2(1,:)-waypoints_2(i,1)).^2 < reach.^2; % is there point close to waypoint
      
      if ~(sum(res) && sum(res2))
         i = i-1;
         break 
      end
      
      waypoint_enter_exit_time_1 = [waypoint_enter_exit_time_1;find(res>0, 1 ),find(res>0, 1, 'last' )]; %#ok<AGROW>
      waypoint_enter_exit_time_2 = [waypoint_enter_exit_time_2;find(res2>0, 1 ),find(res2>0, 1, 'last' )]; %#ok<AGROW>
end

last_waypoint_harvested = i;


%%


min_x1 = min(east_north_1(1,:));
max_x1 = max(east_north_1(1,:));
min_y1 = min(east_north_1(2,:));
max_y1 = max(east_north_1(2,:));

use_waypoint=0;

if use_waypoint
min_x1 = min(waypoints_1(:,1));
max_x1 = max(waypoints_1(:,1));
min_y1 = min(waypoints_1(:,2));
max_y1 = max(waypoints_1(:,2));
end    

lar1 = max([max_y1-min_y1,max_x1-min_x1]);


min_x2 = min(east_north_2(1,:));
max_x2 = max(east_north_2(1,:));
min_y2 = min(east_north_2(2,:));
max_y2 = max(east_north_2(2,:));

use_waypoint=0;

if use_waypoint
min_x2 = min(waypoints_2(:,1));
max_x2 = max(waypoints_2(:,1));
min_y2 = min(waypoints_2(:,2));
max_y2 = max(waypoints_2(:,2));
end    

lar2 = max([max_y2-min_y2,max_x2-min_x2]);

%%
figure
subplot(1,2,1)
axis_max_l = lar1;
axis_min = -20;
axis([min_x1+axis_min min_x1+lar1-axis_min min_y1+axis_min min_y1+lar1-axis_min]);
plot(east_north_1(1,:),east_north_1(2,:))
viscircles(waypoints_1(:,1:2),waypoints_1(:,3))
t=title(['path taken by boat ',FileName1]);
set(t,'Interpreter','none'); 
xlabel('eastern');
ylabel('northen');
subplot(1,2,2)
axis_max_l = lar2;
axis_min = -20;
axis([min_x2+axis_min min_x2+lar2-axis_min min_y2+axis_min min_y2+lar2-axis_min]);
plot(east_north_2(1,:),east_north_2(2,:))
viscircles(waypoints_2(:,1:2),waypoints_2(:,3))
t=title(['path taken by boat ',FileName3]);
set(t,'Interpreter','none'); 
xlabel('eastern');
ylabel('northen');

%%

time_1 = fixtime(time_1);
time_2 = fixtime(time_2);

%plot(time,east_north)

figure
subplot(1,2,1)
plot(time_1,[heading_1;heading2_1])
t=title(['GPS and Compass heading ',FileName1]);
set(t,'Interpreter','none'); 
legend('gps heading','compass heading');
subplot(1,2,2)
plot(time_2,[heading_2;heading2_2])
t=title(['GPS and Compass heading ',FileName3]);
set(t,'Interpreter','none'); 
legend('gps heading','compass heading');



%% v_real

IDX_1 = kmeans([v_1',heading_1',heading2_1'],2);
v_real_1=v_1.*(IDX_1'==1)-v_1.*(IDX_1'~=1);

IDX_2 = kmeans([v_2',heading_2',heading2_2'],2);

v_real_2=v_2.*(IDX_2'==1)-v_2.*(IDX_2'~=1);

v_max = max([v_1,v_2]);
t_max = max([time_1,time_2]);

figure
subplot(1,2,1)
plot(time_1,v_1)
axis([0 t_max 0 v_max])
t=title(['Speed ',FileName1]);
set(t,'Interpreter','none'); 
subplot(1,2,2)
plot(time_2,v_2)
axis([0 t_max 0 v_max])
t=title(['Speed ',FileName3]);
set(t,'Interpreter','none'); 

%% wind
figure
subplot(2,2,1)
plot(time_1,windspeed_1)
t=title(['windspeed ', FileName1]);
set(t,'Interpreter','none'); 
subplot(2,2,2)
plot(time_2,windspeed_2)
t=title(['windspeed ', FileName3]);
set(t,'Interpreter','none'); 
subplot(2,2,3)
plot(time_1,tw_d_1)
t=title(['wind direction ', FileName1]);
set(t,'Interpreter','none'); 
subplot(2,2,4)
plot(time_2,tw_d_2)
t=title(['wind direction ', FileName3]);
set(t,'Interpreter','none'); 


%% analyse waypoint to waypoint


if last_waypoint_harvested>1

mean_v = zeros(2,last_waypoint_harvested-1-deb_waypoints+1);
mean_theta = zeros(2,last_waypoint_harvested-1-deb_waypoints+1);
mean_windspeed = zeros(2,last_waypoint_harvested-1-deb_waypoints+1);
mean_tw_d = zeros(2,last_waypoint_harvested-1-deb_waypoints+1);



   for i=deb_waypoints:last_waypoint_harvested-1
     mean_v(1,i-deb_waypoints+1) = mean(v_1(waypoint_enter_exit_time_1(i-deb_waypoints+1,2):waypoint_enter_exit_time_1(i+1-deb_waypoints+1,1)));  
     mean_v(2,i-deb_waypoints+1) = mean(v_2(waypoint_enter_exit_time_2(i-deb_waypoints+1,2):waypoint_enter_exit_time_2(i+1-deb_waypoints+1,1)));
     mean_theta(1,i-deb_waypoints+1) = mean(yaw_2_1(waypoint_enter_exit_time_1(i-deb_waypoints+1,2):waypoint_enter_exit_time_1(i+1-deb_waypoints+1,1)));  
     mean_theta(2,i-deb_waypoints+1) = mean(yaw_2_2(waypoint_enter_exit_time_2(i-deb_waypoints+1,2):waypoint_enter_exit_time_2(i+1-deb_waypoints+1,1)));
     mean_windspeed(1,i-deb_waypoints+1) = mean(windspeed_1(waypoint_enter_exit_time_1(i-deb_waypoints+1,2):waypoint_enter_exit_time_1(i+1-deb_waypoints+1,1)));  
     mean_windspeed(2,i-deb_waypoints+1) = mean(windspeed_2(waypoint_enter_exit_time_2(i-deb_waypoints+1,2):waypoint_enter_exit_time_2(i+1-deb_waypoints+1,1)));
     mean_tw_d(1,i-deb_waypoints+1) = mean(tw_d_1(waypoint_enter_exit_time_1(i-deb_waypoints+1,2):waypoint_enter_exit_time_1(i+1-deb_waypoints+1,1)));  
     mean_tw_d(2,i-deb_waypoints+1) = mean(tw_d_2(waypoint_enter_exit_time_2(i-deb_waypoints+1,2):waypoint_enter_exit_time_2(i+1-deb_waypoints+1,1)));
       
   end    
end

figure
subplot(4,1,1)
plot(deb_waypoints:last_waypoint_harvested-1,mean_v)
title('Mean of speed between waypoints')
leg = legend(FileName1,FileName3);
set(leg,'Interpreter','none'); 

subplot(4,1,2)
plot(deb_waypoints:last_waypoint_harvested-1,mean_theta)
title('Mean of heading between waypoints')
leg = legend(FileName1,FileName3);
set(leg,'Interpreter','none'); 


subplot(4,1,3)
plot(deb_waypoints:last_waypoint_harvested-1,mean_windspeed)
title('Mean of windspeed between waypoints')
leg = legend(FileName1,FileName3);
set(leg,'Interpreter','none'); 


subplot(4,1,4)
plot(deb_waypoints:last_waypoint_harvested-1,mean_tw_d)
title('Mean of twd between waypoints')
leg = legend(FileName1,FileName3);
set(leg,'Interpreter','none'); 

