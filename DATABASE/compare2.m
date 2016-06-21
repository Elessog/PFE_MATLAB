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
    run_charged = 1;
end

%%


min_x = min(east_north_1(1,:));
max_x = max(east_north_1(1,:));
min_y = min(east_north_1(2,:));
max_y = max(east_north_1(2,:));

use_waypoint=0;

if use_waypoint
min_x = min(waypoints_1(:,1));
max_x = max(waypoints_1(:,1));
min_y = min(waypoints_1(:,2));
max_y = max(waypoints_1(:,2));
end    

lar = max([max_y-min_y,max_x-min_x]);

figure(1)
axis_max_l = lar;
axis_min = -20;
axis([min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min]);
plot(east_north_1(1,:),east_north_1(2,:))
title('path taken by boat');
xlabel('eastern');
ylabel('northen');


time_1 = fixtime(time_1);
time_2 = fixtime(time_2);

%plot(time,east_north)

figure
subplot(1,2,1)
plot(time_1,[heading_1;heading2_1])
title('GPS and Compass heading');
legend('gps heading','compass heading');
subplot(1,2,2)
plot(time_2,[heading_2;heading2_2])
title('GPS and Compass heading');
legend('gps heading','compass heading');

% figure
% subplot(1,2,1)
% plot(time_1,heading_1-heading2_1)
% title('GPS minus Compass heading');
% subplot(1,2,2)
% plot(time_2,heading_2-heading2_2)
% title('GPS minus Compass heading');


%%

%heading_comp_1 = heading_1.*(v_1>=1)+heading2_1.*(v_1<1);
%heading_comp_1 = heading_1.*(v_1>=1)+heading2_1.*(v_1<1);

figure
plot(time_1, [heading_comp_1;heading2_1]);
legend('Change GPS','compass');
subplot(1,2,2)
figure
plot(time_1, [heading_comp_1;heading2_1]);
legend('Change GPS','compass');

%% v_real

IDX_1 = kmeans([v_1',heading_1',heading2_1'],2);
v_real_1=v_1.*(IDX_1'==1)-v_1.*(IDX_1'~=1);

IDX_2 = kmeans([v_2',heading_2',heading2_2'],2);

v_real_2=v_2.*(IDX_2'==1)-v_2.*(IDX_2'~=1);
figure
subplot(1,2,1)
plot(time_1,v_real_1)
subplot(1,2,2)
plot(time_2,v_real_2)



