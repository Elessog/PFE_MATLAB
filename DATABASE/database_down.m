%plot directly from database
% You nee matlab-sqlite3-driver to use this file
%see https://github.com/kyamagu/matlab-sqlite3-driver
% matlab must be started with library preload

if ~exist('database_charged','var')
    database_charged = 0;
else
    choice =questdlg('Would you want to redownload?','Yes','No');
    switch choice
        case 'Yes'
            database_charged = 0;
    end
end

if ~database_charged
    clear all
    [FileName,PathName] = uigetfile('*.db','Select the database');
    
    database = sqlite3.open([PathName,FileName]);
    valid_results = sqlite3.execute(database,'SELECT * FROM datalogs WHERE ((gps_lat IS NOT 0) AND date(gps_time)>date(''1980-12-01''))');
    
    time = zeros(1,length(valid_results));
    east_north = zeros(2,length(valid_results));
    tw_d = zeros(1,length(valid_results));
    heading = zeros(1,length(valid_results));
    v = zeros(1,length(valid_results));
    delta = zeros(2,length(valid_results));
    [X_0,Y_0]=ll2utm(valid_results(1).gps_lat,valid_results(1).gps_lon);
    [Y_s,M_s,D_s,H_s,MN_s,S_s] =datevec(datenum(valid_results(1).gps_time,'yyyy-mm-dd HH:MM:SS'));
    for i=1:length(valid_results)
        current_row = valid_results(i);
        [Y,M,D,H,MN,S] =datevec(datenum(current_row.gps_time,'yyyy-mm-dd HH:MM:SS')) ;
        time(i) = (H-H_s)*3600+(MN-MN_s)*60+(S-S_s);
        [X,Y]=ll2utm(current_row.gps_lat,current_row.gps_lon);
        east_north(:,i)=[X-X_0,Y-Y_0];
        %tw_d(i)=-current_row.twd_calc*pi/180+pi/2;
        heading(i)=-current_row.cps_head*pi/180+pi/2;
        v(i) = current_row.gps_spd;
        delta(:,i) = [(current_row.rc_cmd-5520)*(pi/6)/1500;(current_row.sc_cmd-4215)*(pi/-6.165)/900];
    end
    database_charged = 1;
    sqlite3.close(database);
end

min_x = min(east_north(1,:));
max_x = max(east_north(1,:));
min_y = min(east_north(2,:));
max_y = max(east_north(2,:));

lar = max([max_y-min_y,max_x-min_x]);

figure(1)
plot(east_north(1,:),east_north(2,:))
title('path taken by boat');
xlabel('eastern');
ylabel('northen');


time = fixtime(time);

%plot(time,east_north)



%% visu


index_out=1; %restarting for the controller for visualisation

p1 = 0.05;
a2 = 2;
global q;
q=0;
jk = 0;

%aviobj = avifile('smith.avi','compression','None','fps',10);
%set(figure(666), 'Position', [100, 100, 1120, 840]);
figure(666)
jump = 100;
for i=30000:jump:length(time)-1
    %% boat w
    %figure(668)
    clf
    psi = tw_d(i);
    pos_b = east_north(:,i);
    delta_r = delta(1,i);
    delta_s = delta(2,i);
    %clf         %clear current figure
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    axis square
    axis_max_l = lar;
    axis_min = -20;
    s = axis_max_l*.04;
    axis([min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min]);
    
    %draw wind direction
    m_x = min_x+lar/2;
    m_y = min_y+lar/2;
    x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
    y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
    line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
    line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
    line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
    pointx = [0,20,40,60,30,0];
    pointy = [0,0,20,10,0,0];
    plot(pointx,pointy,'black')
    sing = -sign(sin(heading(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),heading(i),delta_r,sing*delta_s,'g');
    
    title_f = sprintf('Time : %0.2f s theta %0.2f',time(i),heading(i));
    title(title_f);
    if(i+jump<length(time)+1)
      pause((time(i+jump)-time(i))/(jump*10))
    end
 %   aviobj = addframe(aviobj,gcf);
    
    
end
%viobj = close(aviobj);