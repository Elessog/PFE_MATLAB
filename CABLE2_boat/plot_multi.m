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
    [FileName,PathName] = uigetfile('*.mat','Select the MATLAB run without smith');

    load(FileName);
    pos_boat_wt = pos_boat;
    theta_boat_wt = theta_boat;
    v_wt =v; 

    [FileName,PathName] = uigetfile('*.mat','Select the MATLAB run with smith');

    load(FileName);
    run_charged = 1;
end

figure
hold on
axis square
axis([-0 70 -20 40])
plot(pos_boat(:,1),pos_boat(:,2),'g')
pointx = [0,20,40,60,30,0];
pointy = [0,0,20,10,0,0];
plot(pointx,pointy,'black')
plot(pos_boat_wt(:,1),pos_boat_wt(:,2),'r')
legend('with cable','path','without cable')

%% visu


index_out=1; %restarting for the controller for visualisation

p1 = 0.05;
a2 = 2;
figure(666)
jk = 0;
for i=1:10:length(x)-1
    %% boat w
    %figure(668)
    clf
    pos_b = pos_boat(i,:);
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [delta_r, delta_s] = controller_simpleLine_v_control(pos_b(1) ,pos_b(2), theta_boat(i),v(i) ,psi, a, b2);

    %clf         %clear current figure
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    axis square
    axis_max_l = 100;
    axis_min = -20;
    s = axis_max_l*.04;
    axis([axis_min-s axis_min+axis_max_l+s axis_min-s axis_min+axis_max_l+s]);
    
    %draw wind direction
    m_x = axis_min+axis_max_l/2;
    m_y = axis_min+axis_max_l/2;
    x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
    y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
    line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
    line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
    line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
    pointx = [0,20,40,60,30,0];
    pointy = [0,0,20,10,0,0];
    plot(pointx,pointy,'black')
    sing = sign(sin(theta_boat(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),theta_boat(i),delta_r,sing*delta_s);

    %% boat without
    pos_b = pos_boat_wt(i,:);
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [delta_r, delta_s] = controller_simpleLine_v_control(pos_b(1) ,pos_b(2), theta_boat_wt(i),v_wt(i) ,psi, a, b2);      
    sing = sign(sin(theta_boat(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),theta_boat_wt(i),delta_r,sing*delta_s);
    

    title_f = sprintf('Time : %0.3f s theta %0.2f',i*stepH,theta_boat(i));
    title(title_f);

    pause(stepH*1)
    
end
