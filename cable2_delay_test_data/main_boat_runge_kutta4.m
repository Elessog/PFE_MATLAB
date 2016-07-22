clear all;close all;clc;


[FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');

load([PathName,FileName]);

labview_waypoints = 0;
if labview_waypoints
    [FileName2,PathName2] = uigetfile('*.mat','Select the labview waypoints');
    load(FileName2);
    waypoints = [utm_x-origin(1) utm_y-origin(2) (1:length(utm_x))'];
end

[way_lat,way_lont] = utm2ll(waypoints(:,1)+origin(1),waypoints(:,2)+origin(2),34);
[pos_lat,pos_lont] = utm2ll(east_north(1,:)+origin(1),east_north(2,:)+origin(2),34);
origin2=origin;
waypoints_t =waypoints;

global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_dot boat_dotdot L Lg m mg index_out q ...
    windspeed_t psi controller_freq...
    size_rect_cont control_computed delay buffer_command idx_bfc...
    command_buffer_size delta_r delta_s tacking...
     pos_sum active_os delta_s_s delta_r_s;


%% preprocessing of test data
heading_comp = heading.*(v>=1)+heading2.*(v<1);
time = fixtime(time);
accel = diff(v)./diff(time);
old_v = v;

%% 

%%%%%% Time parameters %%%%%%%
stepH = 0.001;
x= time(1):stepH:time(end-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

controller_freq = 4;
size_rect_cont = 0.1;
control_computed = 0;
delay = 2;
command_buffer_size =14; %floor(controller_freq*delay)+10;
buffer_command = zeros(3,command_buffer_size);
idx_bfc = 1;
rode_number = 4;%number of rods to simulate the cable

length_delay = floor(delay/stepH)+1;
active_os = 0;
tacking = 0;



%% initialization of the state of the boat
origin = [east_north(1,1);east_north(2,1);0];
boat_dot=[v(1)*cos(heading_comp(1));v(1)*sin(heading_comp(1));0];
boat_dotdot=[accel(1)*cos(heading_comp(1));accel(1)*cos(heading_comp(1));0];

index_out=1;%#ok<NASGU> %index for the path planning script
q=1; %tacking variable

theta_0 = heading_comp(1);
windspeed_t = windspeed(1);
psi=tw_d(1)+pi;

delta_r = delta(1,1);
delta_s = delta(2,1);
delta_r_s = delta(1,1);
delta_s_s = delta(2,1);

%% Matrix creation for the cable simulation
%b initial vector of the rods
% b_0 = [0 0 0
%     0  0 0
%     -1 -1 -1];
L=1.5*ones(rode_number,1);
Lg = 1.5*ones(rode_number*3,1);

b_0 = zeros(3,rode_number);
b_0(1,:)=-L'.*cos(theta_0).*cos(pi/3).*ones(1,rode_number);
b_0(2,:)=-L'.*sin(theta_0).*cos(pi/3).*ones(1,rode_number);
b_0(3,:)=-L'.*sin(pi/3).*ones(1,rode_number);
dl = 0.5;
m = dl*L;
mg = dl*Lg;
r_0 = zeros(3,rode_number);
r_0(:,1) = b_0(:,1)/2.0+origin;

for i=2:length(b_0(1,:))
    r_0(:,i) = origin+sum(b_0(:,1:i-1),2)+b_0(:,i)/2.0;
end

%vect_dir creation for easy force adding

vect_x = zeros(3*rode_number,1);
vect_y = zeros(3*rode_number,1);
vect_z = zeros(3*rode_number,1);
vect_x(1:3:length(vect_x))=1;
vect_y(2:3:length(vect_y))=1;
vect_z(3:3:length(vect_z))=1;
%creation of q1 and q2
q1 = reshape(r_0,3*rode_number,1);
q2 = reshape(b_0,3*rode_number,1);

%creation of Wn1c

I_blk_n = eye(3);
for i=1:rode_number-1
    I_blk_n=blkdiag(I_blk_n,eye(3));
end

I_blk_n_minus1 = eye(3);
for i=1:rode_number-2
    I_blk_n_minus1=blkdiag(I_blk_n_minus1,eye(3));
end
I_blk_nm1=I_blk_n_minus1;
I_blk_n_minus1 = horzcat(I_blk_n_minus1,zeros(length(I_blk_n_minus1(:,1)),3));
I_blk_n_minus1 = vertcat(zeros(3,length(I_blk_n_minus1(1,:))),I_blk_n_minus1);

Wn1ca=-I_blk_n;
Wn1cb=I_blk_n_minus1;

Wn1c = Wn1ca+Wn1cb;
%mass matrix

%construction of Pn1c
Pn1c=(1/2.0)*(I_blk_n_minus1+I_blk_n);

Nn1c=zeros(3*rode_number,1);
Nn1c(1:3)=origin;

%%% constraint controler coefficients %%
%%% for the cable simulation      %%%%%%
wnl=500;
zeta=1;
Kpl=wnl^2;
Kdl=2*zeta*wnl;
Kil=10000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lambdainverse = ((Wn1c*Wn1c'-6*Pn1c*(Wn1ca'-Wn1cb'))^-1);%unchanging part of the lagrangian multipliers calculation


%% creation vector y for the differential solving


r_dot0 = zeros(size(q1));
r_dot0(1:3:end)=v(1)*cos(heading_comp(1));
r_dot0(2:3:end)=v(1)*sin(heading_comp(1));

y0 = vertcat(q2,q1);
y0 = vertcat(y0,zeros(size(q2)));
y0 = vertcat(y0,r_dot0);
y0 = vertcat(y0,zeros(rode_number,1));
y0 = vertcat(y0,[origin;theta_0;v(1);0;0;0]);


y = zeros(length(y0),length(x));
y(:,1) = y0;                            % initial condition

F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire
sauv_delta = zeros(4,length(x));
pos_sum = [0,0,0,0];
f_cable= zeros(3,length(x));

%% computation of the ODE
for i=1:length(x)-1                            % calculation loop
    
    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_2);
    [k_4,force_cable] = F_xy(x(i)+stepH,y(:,i)+k_3*stepH);
    f_cable(:,i+1)=force_cable;
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
    
    %% delta calc
    time_idx = find(time>x(i+1),1,'first');
    if(mod(i,1000)==0)
        fprintf('Time: %.1f/%d  index: %d  %d\n',time(time_idx)-time(1),time(end-1)-time(1),time_idx,i);
    end
    
    
    windspeed_t = windspeed(time_idx);
    psi=2.3;%tw_d(time_idx);
    sauv_delta(:,i) = [delta_r;delta_s;delta_r_s;delta_s_s];
    delta_r = delta(1,time_idx);
    delta_s = delta(2,time_idx);

end

%%
f_cable=f_cable';


y= y';


%% data reorganization
b = y(:,1:3*rode_number);
r = y(:,3*rode_number+1:2*3*rode_number);
bdot = y(:,2*3*rode_number+1:3*3*rode_number);
rdot = y(:,3*3*rode_number+1:4*3*rode_number);
errorint = y(:,4*3*rode_number+1:4*3*rode_number+rode_number);
pos_boat = y(:,4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);
theta_boat = y(:,4*3*rode_number+rode_number+4);
v =  y(:,4*3*rode_number+rode_number+5);
v_cable =  y(:,4*3*rode_number+rode_number+6:4*3*rode_number+rode_number+7);
theta_dot_boat=  y(:,4*3*rode_number+rode_number+8);

f_cable_frameBoat = f_cable;% Transform the force of the cable to the frame of the boat
for i=1:length(f_cable_frameBoat(:,1))
    f_cable_frameBoat(i,:)=([cos(-theta_boat(i)) -sin(-theta_boat(i)) 0;
        sin(-theta_boat(i)) cos(-theta_boat(i)) 0;
        0 0 1]*(f_cable(i,:)'))';
    
end

errorLdot = zeros(length(x),rode_number);% compute the error of the simulation on the length of the cable
for j=1:length(x)
    for k=1:rode_number
        errorLdot(j,k) = norm(b(j,1+(k-1)*3:k*3))-L(k);
    end
end

%% visu



omega_dot_v = zeros(length(1:100:length(x)-1),4);
cable_drift = zeros(length(1:100:length(x)-1),2);
rod_end  = zeros(length(1:100:length(x)-1),3);
v_down = zeros(length(1:100:length(x)-1),1);
v_dot_c= zeros(length(1:100:length(x)-1),2);
force_v_= zeros(length(1:100:length(x)-1),2);
v_dot_= zeros(length(1:100:length(x)-1),2);
f_rudder_= zeros(length(1:100:length(x)-1),2);
f_sail_= zeros(length(1:100:length(x)-1),2);
f_frict_=  zeros(length(1:100:length(x)-1),2);

index_out=1; %restarting for the controller for visualisation
ratio =5;

min_x = min(pos_boat(:,1));
max_x = max(pos_boat(:,1));
min_y = min(pos_boat(:,2));
max_y = max(pos_boat(:,2));

use_waypoint=0;

if use_waypoint
min_x = min(waypoints(:,1));
max_x = max(waypoints(:,1));
min_y = min(waypoints(:,2));
max_y = max(waypoints(:,2));
end    

lar = max([max_y-min_y,max_x-min_x]);

p1 = 0.05;
a2 = 2;
figure(666)
jk = 0;

for i=1:100:length(x)-1
    %% cable
    jk = jk+1;
    clf
    subplot(1,3,1)
    
    pos_b = pos_boat(i,:);
    l = sum(L);
    axis([-l+pos_b(1) l+pos_b(1)...
        -l+pos_b(2) l+pos_b(2)...
        -l+pos_b(3) 2+pos_b(3)])
    axis vis3d
    l = pos_boat(i,:);
    for number_body=1:rode_number
        
        point=pos_boat(i,:);
        for j=1:number_body
            point = point+b(i,1+(j-1)*3:j*3);
        end
        l = [l; point];
    end
    
    draw_cable(l,666,['r','g','b'])
    
    %% boat
    %figure(668)
    subplot(1,3,2)
    
    [a,b2,index_out] = path_planning_v_control(pos_b(1) ,pos_b(2) ,index_out);
    [force_v_dot,v_dot_main,omega_dot_t,v_dot_cable,f_rudder,f_sail,f_frict] = model_sailboat_jaulin_modified4_visu(y(i,4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+8),a2,psi,delta_s,delta_r,-f_cable(i,:));
    sum_force = (v_dot_main+v_dot_cable)*300;
    omega_dot_v(jk,:) = omega_dot_t;
    rod_end(jk,:) = sum(reshape(b(i,:),3,rode_number),2)';
    cable_drift(jk,:) =v_cable(i,:);
    v_down(jk) =v(i);
    v_dot_c(jk,:) = v_dot_cable;
    force_v_(jk,:) = force_v_dot;
    v_dot_(jk,:) = v_dot_main; 
    f_rudder_(jk,:) = f_rudder;
    f_sail_(jk,:) = f_sail;
    f_frict_(jk,:) = f_frict;
    %clf         %clear current figure
    hold on
    xlabel('x [m]')
    ylabel('y [m]')
    axis square
    axis_max_l = lar;
    s = axis_max_l*.04;
    
    axis_min = -20;
    axis([min_x+axis_min min_x+lar-axis_min min_y+axis_min min_y+lar-axis_min]);

    
    %draw wind direction
    m_x = min_x+axis_min+axis_max_l/2;
    m_y = min_y+axis_min+axis_max_l/2;
    x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
    y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
    line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
    line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
    line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
    %line([a(1) b2(1)],[a(2) b2(2)],'color','black');
    
    %atio = 10;
    %line([pos_b(1)  pos_b(1)+v(i)*cos(theta_boat(i))*ratio],[pos_b(2)  pos_b(2)+v(i)*sin(theta_boat(i))*ratio],'color','b');
    %line([pos_b(1)  pos_b(1)+v(i)*cos(theta_boat(i))*ratio+p1*a2*cos(psi)*ratio+v_cable(1)*ratio],[pos_b(2)  pos_b(2)+v(i)*sin(theta_boat(i))*ratio+p1*a2*sin(psi)*ratio+v_cable(2)*ratio],'color','g');
    
    %
       line([pos_b(1)  pos_b(1)+v_dot_main(1)*ratio],[pos_b(2)  pos_b(2)+v_dot_main(2)*ratio],'color','g');
       line([pos_b(1)-2*cos(theta_boat(i))  pos_b(1)-2*cos(theta_boat(i))+force_v_dot(1)*ratio],...
            [pos_b(2)-2*sin(theta_boat(i))  pos_b(2)-2*sin(theta_boat(i))+force_v_dot(2)*ratio],'color','r');
    %   line([pos_b(1)  pos_b(1)+sum_force(1)*ratio],[pos_b(2)  pos_b(2)+sum_force(2)*ratio],'color','m');
    %
    %
    sing = sign(sin(theta_boat(i)-psi));
    if sing==0
        sing = 1;
    end
    draw_boat([],s,pos_b(1),pos_b(2),theta_boat(i),sauv_delta(3,i),sing*abs(sauv_delta(4,i)),'b');
    viscircles(waypoints(:,1:2),waypoints(:,3));
    %line([x,x+0.05*axis_max_l*cos(alpha_cable)],[y,y+0.05*axis_max_l*sin(alpha_cable)],'color','c')
    %delta_r
    %delta_sMax
    title_f = sprintf('Time : %0.3f s',i*stepH);
    title(title_f);
    
    subplot(1,3,3)
    plot((1:100:length(x)-1)*stepH,omega_dot_v);legend('sail','rudder','friction','cable');
    %plot((1:100:length(x)-1)*stepH,rod_end);
    %plot((1:100:length(x)-1)*stepH,cable_drift);legend('x','y','z');
    pause(stepH*100)
    
end
disp('end');
%%


figure
plot(x(1:100:length(x)-1),[v_dot_ force_v_ f_rudder_ f_sail_ f_frict_])

h=legend('$v_{\dot{x}}$','$v_{\dot{y}}$','$cable_x$','$cable_y$','$f_{rudderX}$','$f_{rudderY}$','$f_{sailX}$','$f_{sailY}$','$f_{frictX}$','$f_{frictY}$');
set(h,'Interpreter','latex')


theta_c = mod(y(1:100:length(x)-1,4*3*rode_number+rode_number+4),2*pi);
figure
v_dot_r = v_dot_(:,1)./cos(theta_c);
force_v_r = force_v_(:,1)./cos(theta_c);
f_rudder_r= f_rudder_(:,1)./cos(theta_c);
f_sail_r= f_sail_(:,1)./cos(theta_c);
f_frict_r= f_frict_(:,1)./cos(theta_c);

plot(x(1:100:length(x)-1),[v_dot_r force_v_r f_rudder_r f_sail_r f_frict_r])
h=legend('$v_{dot}$','$cable$','$f_{rudder}$','$f_{sail}$','$f_{frict}$');
set(h,'Interpreter','latex')

