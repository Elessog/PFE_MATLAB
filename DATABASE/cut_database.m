%save directly from database
% You nee matlab-sqlite3-driver to use this file
%see https://github.com/kyamagu/matlab-sqlite3-driver
% matlab must be started with library preload
% for ubuntu:
% LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/lib/x86_64-linux-gnu/libgcc_s.so.1" matlab -desktop

%this script cut the database into differente mat file corresponding to
%different time


close all;clc;clear all;
[FileName,PathName] = uigetfile('*.db','Select the database');
if FileName==0
    error('File not chosen');
end
database = sqlite3.open([PathName,FileName]);

command = ['SELECT  gps_datalogs.time, gps_datalogs.latitude, gps_datalogs.longitude, gps_datalogs.speed,', ...
        'gps_datalogs.heading,arduino_datalogs.pressure,arduino_datalogs.rudder,', ...
'arduino_datalogs.sheet,arduino_datalogs.current,compass_datalogs.heading,compass_datalogs.pitch,', ...
'compass_datalogs.roll,windsensor_datalogs.direction,windsensor_datalogs.speed,', ...
'course_calculation_datalogs.tack,system_datalogs.true_wind_direction_calc', ...
' FROM  system_datalogs INNER JOIN  arduino_datalogs ON  system_datalogs.arduino_id=arduino_datalogs.id', ...
' INNER JOIN compass_datalogs ON  system_datalogs.compass_id=compass_datalogs.id', ...
' INNER JOIN windsensor_datalogs ON  system_datalogs.windsensor_id=windsensor_datalogs.id', ...
' INNER JOIN  course_calculation_datalogs ON system_datalogs.course_calculation_id=course_calculation_datalogs.id', ...
' INNER JOIN gps_datalogs ON system_datalogs.gps_id=gps_datalogs.id', ...
' WHERE ((gps_datalogs.latitude IS NOT 0) AND date(gps_datalogs.time)>date(''1980-12-01''))'];


valid_results = sqlite3.execute(database,'SELECT * FROM datalogs WHERE ((gps_lat IS NOT 0) AND date(gps_time)>date(''1980-12-01''))');

timestamps = [];

[Y_s,M_s,D_s,H_s,MN_s,S_s] =datevec(datenum(valid_results(1).gps_time,'yyyy-mm-dd HH:MM:SS'));
[X_0,Y_0]=ll2utm(valid_results(1).gps_lat,valid_results(1).gps_lon);

time_long = zeros(1,length(valid_results));
lat_long = zeros(2,length(valid_results));
if exist('tw_d','var')
    tw_d_long = zeros(1,length(valid_results));
end
heading_long = zeros(1,length(valid_results));
v_long = zeros(1,length(valid_results));
delta_long = zeros(2,length(valid_results));


for i=1:length(valid_results)
    current_row = valid_results(i);
    [Year,M,D,H,MN,S] =datevec(datenum(current_row.gps_time,'yyyy-mm-dd HH:MM:SS')) ;
    time_long(i) =(D-D_s)*24*3600+(H-H_s)*3600+(MN-MN_s)*60+(S-S_s);
    lat_long(:,i) = [current_row.gps_lat;current_row.gps_lon];
    if i>1
        if time_long(i)-time_long(i-1)>30 || time_long(i)-time_long(i-1) < 0
            timestamps = [timestamps i-1];
        end
    end
    if exist('tw_d','var')
        tw_d_long(i)=-current_row.twd_calc*pi/180+pi/2;
    end
    heading_long(i)=-current_row.cps_head*pi/180+pi/2;
    v_long(i) = current_row.gps_spd;
    delta_long(:,i) = [(current_row.rc_cmd-5520)*(pi/6)/1500;(current_row.sc_cmd-4215)*(pi/-6.165)/900];
    
end

timestamps = [timestamps length(valid_results)];
[X,Y]=ll2utm(lat_long(1,:),lat_long(2,:));
east_north_long=[X-X_0;Y-Y_0];
sqlite3.close(database);



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
    if exist('tw_d','var')
        tw_d=tw_d_long(i_deb:i_end);
    end
    heading=heading_long(i_deb:i_end);
    v = v_long(i_deb:i_end);
    delta =delta_long(:,i_deb:i_end) ;
    savefile = sprintf('mat-%s-%d.mat',FileName(1:length(FileName)-3),j);
    save(savefile,'time','east_north','heading','v','delta');
end



