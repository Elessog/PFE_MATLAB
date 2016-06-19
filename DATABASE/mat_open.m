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
    
    run_charged = 1;
end



min_x = min(east_north(1,:));
max_x = max(east_north(1,:));
min_y = min(east_north(2,:));
max_y = max(east_north(2,:));

use_waypoint=0;

if use_waypoint
min_x = min(waypoints(:,1));
max_x = max(waypoints(:,1));
min_y = min(waypoints(:,2));
max_y = max(waypoints(:,2));
end    

lar = max([max_y-min_y,max_x-min_x]);

figure(1)
axis_max_l = lar;
axis_min = -20;
axis([min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min]);
plot(east_north(1,:),east_north(2,:))
title('path taken by boat');
xlabel('eastern');
ylabel('northen');


time = fixtime(time);

%plot(time,east_north)

figure
plot(time,[heading;heading2])
title('GPS and Compass heading');
legend('gps heading','compass heading');

figure
plot(time,[heading-heading2])
title('GPS minus Compass heading');
%legend('diffe');


figure
plot(v,[heading-heading2],'x')
title('GPS minus Compass heading function of speed');
%legend('diffe');


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
jump = 1;
for i=30:jump:length(time)-1
    %% boat w
    %figure(668)
    clf
    if exist('tw_d','var')
        psi = tw_d(i);
    else
        psi = 0;
    end
    pos_b = east_north(:,i);
    if exist('delta','var')
        delta_r = delta(1,i);
        delta_s = delta(2,i);
    else
        delta_r=0;
        delta_s=0;
    end
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
%     x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
%     y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
%     line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
%     line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
%     line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
%     pointx = [0,20,40,60,30,0];
%     pointy = [0,0,20,10,0,0];
%     plot(pointx,pointy,'black')
    sing = -sign(sin(heading(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),heading2(i),delta_r,sing*delta_s,'g');
    viscircles(waypoints(:,1:2),waypoints(:,3));
    
    title_f = sprintf('Time : %0.2f s theta %0.2f',time(i),heading2(i));
    title(title_f);
    if(i+jump<length(time)+1)
      pause((time(i+jump)-time(i))/(jump*10))
    end
 %   aviobj = addframe(aviobj,gcf);
    
    
end
%viobj = close(aviobj);