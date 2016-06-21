%save directly from database
% You nee matlab-sqlite3-driver to use this file
%see https://github.com/kyamagu/matlab-sqlite3-driver
% matlab must be started with library preload
% for ubuntu:
% LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/lib/x86_64-linux-gnu/libgcc_s.so.1" matlab -desktop

%this script cut the database into differente mat file corresponding to
%different time


close all;clc;clear all;
[FileName,PathName] = uigetfile({'*.db';'*.sqlite'},'Select the database');
if FileName==0
    error('File not chosen');
end
database = sqlite3.open([PathName,FileName]);

command = ['SELECT  gps_dataLogs.time, gps_dataLogs.latitude, gps_dataLogs.longitude, gps_dataLogs.speed,', ...
        'gps_dataLogs.heading,', ...
'compass_dataLogs.heading,compass_dataLogs.pitch,', ...
'compass_dataLogs.roll,windsensor_dataLogs.direction,windsensor_dataLogs.speed,', ...
'course_calculation_dataLogs.tack,system_dataLogs.true_wind_direction_calc,', ...
'system_dataLogs.sail_command_sail,system_dataLogs.rudder_command_rudder', ...
' FROM  system_dataLogs ', ...
' INNER JOIN compass_dataLogs ON  system_dataLogs.id_system=compass_dataLogs.id_compass_model', ...
' INNER JOIN windsensor_dataLogs ON  system_dataLogs.id_system=windsensor_dataLogs.id_windsensor', ...
' INNER JOIN  course_calculation_dataLogs ON system_dataLogs.id_system=course_calculation_dataLogs.id_course_calculation', ...
' INNER JOIN gps_dataLogs ON system_dataLogs.id_system=gps_dataLogs.id_gps', ...
' WHERE ((gps_dataLogs.latitude IS NOT 0) AND date(gps_dataLogs.time)>date(''1980-12-01''))'];



valid_results = sqlite3.execute(database,command);

timestamps = [];

[Y_s,M_s,D_s,H_s,MN_s,S_s] =datevec(datenum(valid_results(1).time,'HH:MM:SS'));
[X_0,Y_0]=ll2utm(valid_results(1).latitude,valid_results(1).longitude);

time_long = zeros(1,length(valid_results));
lat_long = zeros(2,length(valid_results));
if  isfield(valid_results,'true_wind_direction_calc')
    tw_d_long = zeros(1,length(valid_results));
end
heading_long = zeros(1,length(valid_results));
heading_2 = zeros(1,length(valid_results));
v_long = zeros(1,length(valid_results));
delta_long = zeros(2,length(valid_results));


for i=1:length(valid_results)
    current_row = valid_results(i);
    [Year,M,D,H,MN,S] =datevec(datenum(current_row.time,'HH:MM:SS')) ;
    time_long(i) =(D-D_s)*24*3600+(H-H_s)*3600+(MN-MN_s)*60+(S-S_s);
    lat_long(:,i) = [current_row.latitude;current_row.longitude];
    if i>1
        if time_long(i)-time_long(i-1)>30 || time_long(i)-time_long(i-1) < 0
            timestamps = [timestamps i-1];
        end
    end
    if isfield(valid_results,'true_wind_direction_calc')
        tw_d_long(i)=-current_row.true_wind_direction_calc*pi/180+pi/2;
    end
    heading_long(i)=-current_row.heading*pi/180+pi/2;
    heading_2(i)=-current_row.heading_1*pi/180+pi/2;
    v_long(i) = current_row.speed;
    delta_long(:,i) = [(current_row.rudder_command_rudder-5520)*(pi/6)/1500;(current_row.sail_command_sail-4215)*(pi/-6.165)/900];
    
end

timestamps = [timestamps length(valid_results)];
[X,Y]=ll2utm(lat_long(1,:),lat_long(2,:));
east_north_long=[X-X_0;Y-Y_0];


%% database waypoints
waypoints_sql = sqlite3.execute(database,'SELECT * from waypoints');

way_lat = zeros(1,length(waypoints_sql));
way_lon = zeros(1,length(waypoints_sql));
radius =  zeros(1,length(waypoints_sql));

for i=1:length(waypoints_sql)
   way_lat(i) = waypoints_sql(i).latitude;
   way_lon(i) = waypoints_sql(i).longitude;
   radius(i) = waypoints_sql(i).radius;
end

[wayX,wayY]=ll2utm(way_lat,way_lon);

waypoints = [wayX-X_0;wayY-Y_0;1:length(waypoints_sql)]';

%% close database
sqlite3.close(database);

%% cutting 

for j=1:length(timestamps)
    if (j==1)
        i_deb = 1;
    else
        i_deb = timestamps(j-1)+1;
    end
    i_end = timestamps(j);
    length_vector = i_end-i_deb+1;
    time = time_long(i_deb:i_end)-time_long(i_deb);
    east_north =[east_north_long(1,i_deb:i_end)-east_north_long(1,i_deb);...
        east_north_long(2,i_deb:i_end)-east_north_long(2,i_deb)];
    if isfield(valid_results,'true_wind_direction_calc')
        tw_d=tw_d_long(i_deb:i_end);
    end
    heading=heading_long(i_deb:i_end);
    heading2 = heading_2(i_deb:i_end);
    yaw_2 = -(heading2 +pi/2)*180/pi;
    yaw = -(heading +pi/2)*180/pi;
    v = v_long(i_deb:i_end);
    delta =delta_long(:,i_deb:i_end) ;
    savefile = sprintf('mat-%s-%d.mat',FileName(1:length(FileName)-3),j);
    origin = east_north_long(:,i_deb)+[X_0;Y_0];
    if isfield(valid_results,'true_wind_direction_calc')
        save(savefile,'time','east_north','heading','heading2','v','yaw','yaw_2','waypoints','origin','delta','tw_d');
    else
        save(savefile,'time','east_north','heading','heading2','v','yaw','yaw_2','waypoints','origin','delta');
    end
end



