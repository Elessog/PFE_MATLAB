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
    [FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');
    
    load(FileName);

    labview_wayponts = 0;
    if labview_wayponts
        [FileName2,PathName2] = uigetfile('*.mat','Select the labview waypoints');
        load(FileName2);
        waypoints = [utm_x-origin(1) utm_y-origin(2) (1:length(utm_x))'];
        
        
    end
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
    
    [FileName3,PathName3] = uigetfile('*.mat','Select the MATLAB run');
    
    load(FileName3);

    labview_wayponts = 0;
    if labview_wayponts
        [FileName2,PathName2] = uigetfile('*.mat','Select the labview waypoints');
        load(FileName2);
        waypoints = [utm_x-origin(1) utm_y-origin(2) (1:length(utm_x))'];
        
        
    end
    waypoints_2 = waypoints;
    time_2 = time;
	east_north_2 = east_north;
	heading_2 = heading;
	heading2_2= heading2;
	yaw_2_2 = yaw_2;
	yaw_2 = yaw;
	v_2 = v;
    windspeed_2 = windspeed;
    tw_d_2 = tw_d;
    
    run_charged = 1;
end

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
figure(1)
subplot(1,2,1)
axis_max_l = lar1;
axis_min = -20;
axis([min_x1+axis_min min_x1+lar1-axis_min min_y1+axis_min min_y1+lar1-axis_min]);
plot(east_north_1(1,:),east_north_1(2,:))
viscircles(waypoints_1(:,1:2),waypoints_1(:,3))
t=title(['path taken by boat ',FileName]);
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
t=title(['GPS and Compass heading ',FileName]);
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
t=title(['Speed ',FileName]);
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
t=title(['windspeed ', FileName]);
set(t,'Interpreter','none'); 
subplot(2,2,2)
plot(time_2,windspeed_2)
t=title(['windspeed ', FileName3]);
set(t,'Interpreter','none'); 
subplot(2,2,3)
plot(time_1,tw_d_1)
t=title(['wind direction ', FileName]);
set(t,'Interpreter','none'); 
subplot(2,2,4)
plot(time_2,tw_d_2)
t=title(['wind direction ', FileName3]);
set(t,'Interpreter','none'); 




