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
    if exist('sauv_fs_vdot','var')
        sauv_fs_vdot_wt = sauv_fs_vdot;
    end
    if exist('sauv_delta','var')
        sauv_delta_wt = sauv_delta;
    end
    if exist('sauv_phi_ap','var')
        sauv_phi_ap_wt = sauv_phi_ap;
    end
    if exist('sauv_w_ap','var')
        sauv_w_ap_wt = sauv_w_ap;
    end
    [FileName,PathName] = uigetfile('*.mat','Select the MATLAB run with smith');
    
    load(FileName);
    run_charged = 1;
end

figure
hold on
axis square
axis([-10 100 -10 100])
plot(pos_boat(:,1),pos_boat(:,2),'g')
pointx = [0,20,40,60,30,0];
pointy = [0,0,20,10,0,0];
plot(pointx,pointy,'black')
plot(pos_boat_wt(:,1),pos_boat_wt(:,2),'r')
plot(y_s(1,:),y_s(2,:),'b')
legend('with smith','path','without smith')

if exist('sauv_fs_vdot','var')
    figure
    hold on
    plot(x,[sauv_fs_vdot;sauv_fs_vdot_wt]);
    hold off
    h = legend('$\dot{v}_\phi$','$fs_\phi$','$\dot{v}_\textnormal{old}$'...
        ,'$fs_\textnormal{old}$'); 
    set(h,'Interpreter','latex')
end

if exist('sauv_delta','var')
    figure
    hold on
    plot(x,[sauv_delta;sauv_delta_wt]);
    hold off
    h = legend('$\delta_{r\phi}$','$\delta_{s\phi}$','$\delta_{r\textnormal{old}}$'...
        ,'$\delta_{s\textnormal{old}}$'); 
    set(h,'Interpreter','latex')
end
% 
if exist('sauv_phi_ap','var')
    figure
    hold on
    plot(x,[sauv_phi_ap;sauv_phi_ap_wt]);
    hold off
    h = legend('$\phi_{ap\phi}$','$\phi_{ap\textnormal{old}}$');
    set(h,'Interpreter','latex')
end

figure
hold on
plot(x,[v,v_wt]);
hold off
h = legend('$v_{\phi}$','$v_{\textnormal{old}}$');
set(h,'Interpreter','latex')

if exist('sauv_w_ap','var')
    figure
    hold on
    plot(x,[sauv_w_ap;sauv_w_ap_wt]);
    hold off
    h = legend('$W_{x\phi}$','$W_{y\phi}$','$W_{x\textnormal{old}}$','$W_{y\textnormal{old}}$');
    set(h,'Interpreter','latex')
end


%% visu


index_out=1; %restarting for the controller for visualisation

p1 = 0.05;
a2 = 2;

jk = 0;

%aviobj = avifile('smith.avi','compression','None','fps',10);
%set(figure(666), 'Position', [100, 100, 1120, 840]);
figure(666)
for i=1:10:length(x)-1
    %% boat w
    %figure(668)
    clf
    pos_b = pos_boat(i,:);
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [delta_r, delta_s] = controller_phi_a(pos_b(1) ,pos_b(2), theta_boat(i),v(i) ,psi, a, b2,3);

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
    draw_boat([],s,pos_b(1),pos_b(2),theta_boat(i),delta_r,sing*delta_s,'g');

    %% boat without
    pos_b = pos_boat_wt(i,:);
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [delta_r, delta_s] = controller_simpleLine(pos_b(1) ,pos_b(2), theta_boat_wt(i),v_wt(i) ,psi, a, b2);      
    sing = sign(sin(theta_boat(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),theta_boat_wt(i),delta_r,sing*delta_s,'r');
    
    title_f = sprintf('Time : %0.3f s theta %0.2f',i*stepH,theta_boat(i));
    title(title_f);
    pause(stepH*1)
 %   aviobj = addframe(aviobj,gcf);
    
    
end
%viobj = close(aviobj);