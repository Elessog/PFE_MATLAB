%plot

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
    [FileName1,PathName] = uigetfile('*.mat','Select the MATLAB run without cable');

    load([PathName,FileName1]);
    x_wt =x;
    pos_boat_wt = pos_boat;
    sauv_delta_wt = sauv_delta;
    theta_boat_wt = theta_boat;
    v_wt =v; 
    waypoints_1 = waypoints;
    time_1 = time;
    windspeed_1 = windspeed;
    tw_d_1 = tw_d;

    [FileName2,PathName] = uigetfile('*.mat','Select the MATLAB run with cable');

    load([PathName,FileName2]);
    waypoints_2 = waypoints;
    time_2 = time;
    windspeed_2 = windspeed;
    tw_d_2 = tw_d;
    
    run_charged = 1;
end

size_waypoints = length(waypoints_1(:,1));
waypoint_enter_exit_time_1 = [];
waypoint_enter_exit_time_2 = [];

for i = 1:size_waypoints
      res = (pos_boat_wt(:,2)-waypoints_1(i,2)).^2+(pos_boat_wt(:,2)-waypoints_1(i,2)).^2 < 100; % is there point close to waypoint
      res2 = (pos_boat(:,2)-waypoints_2(i,2)).^2+(pos_boat(:,1)-waypoints_2(i,1)).^2 < 100; % is there point close to waypoint
      
      if ~(sum(res) && sum(res2))
         break 
      end
      
      waypoint_enter_exit_time_1 = [waypoint_enter_exit_time_1;find(res>0, 1 ),find(res>0, 1, 'last' )]; %#ok<AGROW>
      waypoint_enter_exit_time_2 = [waypoint_enter_exit_time_2;find(res2>0, 1 ),find(res2>0, 1, 'last' )]; %#ok<AGROW>
end

last_waypoint_harvested = i;

%%
figure
hold on
axis square
subplot(1,2,1)
plot(pos_boat(:,1),pos_boat(:,2),'g')
viscircles(waypoints_2(:,1:2),waypoints_2(:,3))
legend('with cable')
subplot(1,2,2)
plot(pos_boat_wt(:,1),pos_boat_wt(:,2),'r')

viscircles(waypoints_1(:,1:2),waypoints_1(:,3))
legend('without cable')

figure
hold on
plot(x,sauv_delta(1,:),'g')
plot(x_wt,sauv_delta_wt(1,:),'r')
title('rudder command with cableor not')
legend('with cable','without cable')

figure
hold on
plot(x,sauv_delta(2,:),'g')
plot(x_wt,sauv_delta_wt(2,:),'r')
title('sail command with cable or not')
legend('with cable','without cable')

figure
subplot(2,2,1)
plot(x,theta_boat)
t=title(['theta ', FileName2]);
set(t,'Interpreter','none'); 
subplot(2,2,2)
plot(x_wt,theta_boat_wt)
t=title(['theta ', FileName1]);
set(t,'Interpreter','none'); 
subplot(2,2,3)
plot(x,v)
t=title(['speed ', FileName2]);
set(t,'Interpreter','none'); 
subplot(2,2,4)
plot(x_wt, v_wt)
t=title(['speed ', FileName1]);
set(t,'Interpreter','none'); 

%% wind
figure
subplot(2,2,2)
plot(time_1,windspeed_1)
t=title(['windspeed ', FileName]);
set(t,'Interpreter','none'); 
subplot(2,2,1)
plot(time_2,windspeed_2)
t=title(['windspeed ', FileName2]);
set(t,'Interpreter','none'); 
subplot(2,2,4)
plot(time_1,tw_d_1)
t=title(['wind direction ', FileName]);
set(t,'Interpreter','none'); 
subplot(2,2,3)
plot(time_2,tw_d_2)
t=title(['wind direction ', FileName2]);
set(t,'Interpreter','none'); 


%% analyse waypoint to waypoint


if last_waypoint_harvested>1

mean_v = zeros(2,last_waypoint_harvested-1);
mean_theta = zeros(2,last_waypoint_harvested-1);
    
   for i=1:last_waypoint_harvested-2
     mean_v(1,i) = mean_fix(v_wt(waypoint_enter_exit_time_1(i,2):waypoint_enter_exit_time_1(i+1,1))*10000)/10000;  
     mean_v(2,i) = mean(v(waypoint_enter_exit_time_2(i,2):waypoint_enter_exit_time_2(i+1,1)));
     mean_theta(1,i) = mean_fix(theta_boat_wt(waypoint_enter_exit_time_1(i,2):waypoint_enter_exit_time_1(i+1,1)));  
     mean_theta(2,i) = mean(theta_boat(waypoint_enter_exit_time_2(i,2):waypoint_enter_exit_time_2(i+1,1)));
    
   end    
end

figure
subplot(2,1,1)
plot(1:last_waypoint_harvested-1,mean_v)

subplot(2,1,2)
plot(1:last_waypoint_harvested-1,mean_theta)
