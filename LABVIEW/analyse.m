close all

if ~exist('run_charged','var')
    run_charged = 0;
else
    choice =questdlg('Would you want to recharge data?','Yes','No');
    switch choice
        case 'Yes'
            run_charged = 0;
    end
end

if ~run_charged
    clear all
    [FileName,PathName] = uigetfile('*.txt','Select the MATLAB run');

    [LabView_time,second,RPI_time,winddir,windspeed,yaw,pitch,roll,accx,accy,accz,...
    pressure,rudder_act,sail_act,current,latitude,longitude,cog,sog,...
    rudderpos,sailpos] = importfile([PathName,FileName]);

%% chosing non corrupt data
    index1 = find(latitude<61 & latitude>59); 
    index2 = find(pressure~=65535);
    index3 = find(longitude<22 & longitude>18); 
    if isempty(index2)
       index2 = find(pressure==65535);
    end
    index = intersect(index1,index2);
    index = intersect(index,index3);
    
    LabView_time =LabView_time(index);
    RPI_time = RPI_time(index);
    winddir =winddir(index);
    windspeed = windspeed(index);
    yaw1 = -yaw(index)*pi/180+pi/2;
    pitch =pitch(index);
    roll =roll(index);
    accx = accx(index);
    accy = accy(index);
    accz =accz(index);
    pressure = pressure(index);
    rudder_act = rudder_act(index);
    sail_act = sail_act(index);
    current = current(index);
    latitude = latitude(index);
    longitude = longitude(index);
    cog = -cog(index)*pi/180+pi/2;
    sog = sog(index);
    rudderpos = (rudderpos(index)-5520)*(pi/6)/1500;
    sailpos = (sailpos(index)-4215)*(pi/-6.165)/900;
    %rudderCMD = [rudderCMD(index(1:end-1));rudderCMD(end)];
    %sailCMD =[sailCMD(index(1:end-1));sailCMD(end)];
    
 %% pre-processing
    [Y_l,M_l,D_l,H_l,MN_l,S_l] =datevec(datenum(LabView_time,'yyyy-mm-dd HH:MM:SS')) ;
    [Y,M,D,H,MN,S] =datevec(datenum(RPI_time,'yyyy-mm-dd HH:MM:SS')) ;
    rpi_t =(H-H(1))*3600+(MN-MN(1))*60+(S-S(1));
    lab_t =(H_l-H_l(1))*3600+(MN_l-MN_l(1))*60+(S_l-S_l(1));
    rpi_t = fixtime(rpi_t);
    lab_t = fixtime(lab_t);
    
    
    
    [utm_x,utm_y] = ll2utm(latitude,longitude);
    position = [utm_x-utm_x(1),utm_y-utm_y(1)];
    
   
%% google earth view
    description ='postion of boat';
    name = 'Test-titre';
    filename = [FileName(1:end-3),'kml'];
    kmlwriteline(filename, latitude, longitude, ...
           'Description', description, 'Name', name);
   
%% end
    run_charged = 1;
end

%% cutting 

timestamps = [];

for i=1:length(rpi_t)
  if i>1
        if rpi_t(i)-rpi_t(i-1)>30 || rpi_t(i)-rpi_t(i-1) < 0
            timestamps = [timestamps i-1];
        end
  end
end

for j=1:length(timestamps)
    if (j==1)
        i_deb = 1;
    else
        i_deb = timestamps(j-1)+1;
    end
    i_end = timestamps(j);
    length_vector = i_end-i_deb+1;
    time = rpi_t(i_deb:i_end)-rpi_t(i_deb);
    east_north =[position(i_deb:i_end,1)-position(i_deb,1) ...
        position(i_deb:i_end,2)-position(i_deb,2)]';
    heading=cog(i_deb:i_end);
    heading2 = yaw1(i_deb:i_end);
    yaw_2 = -(heading2 +pi/2)*180/pi;
    yaw = -(heading +pi/2)*180/pi;
    v = sog(i_deb:i_end);
    delta =[rudderpos(i_deb:i_end)'; sailpos(i_deb:i_end)'];
    savefile = sprintf('mat-%s-%d.mat',FileName(1:length(FileName)-4),j);
    origin = position(i_deb,:)'+[utm_x(1);utm_y(1)];
    press = pressure(i_deb:i_end);
    rudder = rudder_act(i_deb:i_end);
    sail = sail_act(i_deb:i_end);
    curr = current(i_deb:i_end);
    save(savefile,'time','east_north','heading','heading2','v','yaw','yaw_2','origin','delta','press','rudder','sail','curr');

end



%%

% delay on command from labview
figure
plot(rpi_t,[rudderpos,rpi_t]);
xlabel('time')
ylabel('command value')
legend('command labview','command on py','time');
title('Delay on command from labview');


figure
plot(position(:,1),position(:,2));

figure
plot(sog,pressure,'x');
xlabel('speed')
ylabel('pressure value')
title('pressure by speed');


figure
plot(rpi_t,[pressure/100,sog]);
xlabel('time')
ylabel('speed and pressure/100')
legend('pressure','speed');
   

figure
plot(rpi_t,[yaw1,cog,sog*100]);
xlabel('time')
ylabel('heading by compass and gps and speed')
legend('compass heading','gps heading','speed');


   
