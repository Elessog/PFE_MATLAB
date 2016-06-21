%plot
global axis_mat time_g east_north_g v_g i_deb i_end ;
close all;clear all;
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
    global axis_mat time_g east_north_g v_g i_deb i_end ; %#ok<TLEV,REDEF>
    load(FileName);
    i_deb = 1;
    i_end = length(v);
    
    run_charged = 1;
end

east_north_g = east_north;
v_g = v;

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

axis_max_l = lar;
axis_min = -20;
axis_mat = [min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min];

figure(100)
set(figure(100), 'Position', [10, 200, 2000, 1100], 'toolbar', 'figure' )
subplot(1,2,1)

plot(east_north(1,:),east_north(2,:));
axis square
axis(axis_mat);
xlabel('eastern');
ylabel('northen');

subplot(1,2,2)

time = fixtime(time);
time_g=time;
plot(time,v)


set(figure(100), 'Position', [10, 200, 2000, 1100], 'toolbar', 'figure' )
hd = uicontrol('style','slider','Min',1,'Max',length(v),'Value',i_deb,'units','pixel','position',[100 50 1800 30]);
he = uicontrol('style','slider','Min',1,'Max',length(v),'Value',i_end,'units','pixel','position',[100 10 1800 30]);

printdeb(hd, 0);
printend(he, 0);


lise = addlistener(he,'ContinuousValueChange',@(hObject, event) printend(hObject, event));
lisd = addlistener(hd,'ContinuousValueChange',@(hObject, event) printdeb(hObject, event));


prompt = {'FileName:'};
dlg_title = 'Save file under name';
num_lines = 1;
defaultAns = sprintf('short-%s',FileName(1:length(FileName)-4)) ;
defaultans = {defaultAns};

options.Resize='on';
options.WindowStyle='normal';
options.Interpreter='tex';

answer = inputdlg(prompt,dlg_title,num_lines,defaultans,options);

text = char(answer(1));

time = time(i_deb:i_end);
east_north =east_north(:,i_deb:i_end);
if exist('tw_d','var')
    tw_d=tw_d(i_deb:i_end);
end
heading=heading(i_deb:i_end);
heading2 = heading2(i_deb:i_end);
yaw_2 =yaw_2(i_deb:i_end);
yaw = yaw(i_deb:i_end);
v = v(i_deb:i_end);
delta =delta(:,i_deb:i_end) ;
save([text,'.mat'],'time','east_north','heading','heading2','v','yaw','yaw_2','waypoints','origin','delta');
clear answer

