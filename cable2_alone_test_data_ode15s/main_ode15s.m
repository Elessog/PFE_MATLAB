clear all;close all;clc;

global east_north_real;

[FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');

load([PathName,FileName]);


[way_lat,way_lont] = utm2ll(waypoints(:,1)+origin(1),waypoints(:,2)+origin(2),34);
[pos_lat,pos_lont] = utm2ll(east_north(1,:)+origin(1),east_north(2,:)+origin(2),34);

global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_pos boat_dot boat_dotdot;
global L Lg mg m coeff_div_pressure_sensor coefSpring coefDotSpring;
global  accel time heading_comp v_real boolPrint;
rode_number = 3;%number of rods to simulate the cable
coeff_div_pressure_sensor = 3;
length_cable = 9;
coefSpring =0;%2;
coefDotSpring = -18;%-10;
boolPrint = 1;

%% preprocessing of test data
size_buff = 5;

heading_comp = correction_angle(heading2,5);%heading.*(v>=1)+heading2.*(v<1);
IDX = kmeans([v',heading',(heading2-heading)'],2);
v_real_2=v.*(IDX'==1)-v.*(IDX'~=1);

idx1 = abs(wrapTo2Pi(heading-heading2))>pi/2;
idx2 = abs(wrapToPi(heading-heading2))>pi/2;
idx = idx1 | idx2;
v_real = v.*(idx==0)-v.*(idx==1);
v_real=v;
time = fixtime(time);
time =time-time(1);


east_north_real=zeros(2,length(east_north(1,:)));
for i=1:length(east_north(1,:))
    east_north_real(1,i) = mean(east_north(1,max(1,i-size_buff):min(i+size_buff,length(east_north(1,:)))));
    east_north_real(2,i) = mean(east_north(2,max(1,i-size_buff):min(i+size_buff,length(east_north(1,:)))));
end

v_r=sqrt((diff(east_north_real(1,:))./diff(time)).^2+(diff(east_north_real(2,:))./diff(time)).^2);
v_real = zeros(1,length(v));
for i=1:length(v)
    v_real(i) = mean(v_r(max(1,i-size_buff):min(i+size_buff,length(v_r))));
end

v_t = v_real;
v_real = zeros(1,length(v));
for i=1:length(v)
    v_real(i) = mean(v_t(max(1,i-size_buff):min(i+size_buff,length(v))));
end


accel1 = diff(v_real)./diff(time);

size_buff = 10;
accel = zeros(1,length(accel));
for i=1:length(accel1)
    accel(i) = mean(accel1(max(1,i-size_buff):min(i+size_buff,length(accel1))));
end

length_cable_press = 3;
press = arduino(1,:);
pressure = [0,181;1,244;2,302;3,355];
press_norm = -lagrange(press,pressure(:,2)',pressure(:,1)');
press_norm_0 = press_norm(1);
angle_cable = asin(-press_norm_0/3);


%% initialization of the state of the boat

boat_pos = [east_north_real(1,1);east_north_real(2,1);-0.30];
boat_dot=[v_real(1)*cos(heading_comp(1));v_real(1)*sin(heading_comp(1));0];
boat_dotdot=[accel(1)*cos(heading_comp(1));accel(1)*cos(heading_comp(1));0];
%theta_0 = heading_comp;
%% Matrix creation for the cable simulation
%b initial vector of the rods
% b_0 = [0 0 0
%     0  0 0
%     -1 -1 -1];
L=(length_cable/rode_number)*ones(rode_number,1);
%L(rode_number) = 0.1;
Lg = (length_cable/rode_number)*ones(rode_number*3,1);
%Lg(rode_number*3-2:rode_number*3) = 4;
b_0 = zeros(3,rode_number);
b_0(1,:)=-L'.*cos(heading_comp(1))*cos(angle_cable).*ones(1,rode_number);
b_0(2,:)=-L'.*sin(heading_comp(1))*cos(angle_cable).*ones(1,rode_number);
b_0(3,:) = -L'*sin(angle_cable).*ones(1,rode_number);

dl = 0.130;%linear mass of the cable
m = dl*L;%mass of rods
mg = dl*Lg;% mass of every direction

pressure_sensor_pos=(rode_number/coeff_div_pressure_sensor-1)*3+1:(rode_number/coeff_div_pressure_sensor-1)*3+3;
pressure_sensor_cable = 1:pressure_sensor_pos(end);

m(rode_number/coeff_div_pressure_sensor) =m(rode_number/coeff_div_pressure_sensor)+0.300;
m(1:rode_number/coeff_div_pressure_sensor) =m(1:rode_number/coeff_div_pressure_sensor)+0.010*L(1:rode_number/coeff_div_pressure_sensor);

for idx = 1:rode_number/coeff_div_pressure_sensor
   mg(1+(idx-1)*3:3*idx)=m(idx) ;
end

%for 10m cable%%%%%%%
mg((rode_number-1)*3+1:rode_number*3) = mg((rode_number-1)*3+1:rode_number*3)+dl*1;
%%%%%%%%%%%%%%%%%%%%%


m(rode_number) = m(rode_number)+dl*1;

r_0 = zeros(3,rode_number);
r_0(:,1) = b_0(:,1)/2.0+boat_pos;

for i=2:length(b_0(1,:))
    r_0(:,i) = boat_pos+sum(b_0(:,1:i-1),2)+b_0(:,i)/2.0;%positionning boat
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
Nn1c(1:3)=boat_pos;

%%% constraint controler coefficients %%
%%% for the cable simulation      %%%%%%
wnl=1000;
zeta=1;
Kpl=wnl^2;
Kdl=2*zeta*wnl;
Kil=10000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lambdainverse = ((Wn1c*Wn1c'-6*Pn1c*(Wn1ca'-Wn1cb'))^-1);%unchanging part of the lagrangian multipliers calculation


%% creation vector y for the differential solving
y0 = vertcat(q2,q1);
y0 = vertcat(y0,zeros(size(q2)));
y0 = vertcat(y0,zeros(size(q1)));
y0 = vertcat(y0,zeros(rode_number,1));
y0 = vertcat(y0,[boat_pos;0;0;0]);

%%%%%% Time parameters %%%%%%%
stepH = 0.1;
x= time(1):stepH:20;%time(end-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire

f_cable= zeros(3,length(x));

%% computation of the ODE
[t,y] = ode45(@boat_cable_simulation4,x,y0);

%% data reorganization

b = y(:,1:3*rode_number);
r = y(:,3*rode_number+1:2*3*rode_number);
bdot = y(:,2*3*rode_number+1:3*3*rode_number);
rdot = y(:,3*3*rode_number+1:4*3*rode_number);
errorint = y(:,4*3*rode_number+1:4*3*rode_number+rode_number);
pos_boat = y(:,4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);
% theta_boat = y(:,4*3*rode_number+rode_number+4);
% v =  y(:,4*3*rode_number+rode_number+5);
% v_cable =  y(:,4*3*rode_number+rode_number+6:4*3*rode_number+rode_number+7);
% theta_dot_boat=  y(:,4*3*rode_number+rode_number+8);

%% computation of forces

%fa and fb construction


rdotdot_ = diff(rdot)/stepH;
radius = 0.005;

f_cable=zeros(length(x),3);
for i=2:length(rdot(:,1))
  P_Arch = ((-mg*9.81+1000*9.81*pi*Lg*radius^2)).*vect_z;
  FluidFriction = (-1.2*2*radius*abs(b(i,:))*1000/2).*rdot(i,:).*abs(rdot(i,:));
  fa=zeros(3*rode_number,1)+P_Arch+FluidFriction';
  fb=fa;
  tau1 = (fa+fb)./mg;
  tauc1 = rdotdot_(i-1,:)'-tau1;
  tauc1r = reshape(mg.*tauc1,3,rode_number);
  f_cable(i,:) =sum(tauc1r,2)';
end


accel_x= zeros(size(x));
comp_x = zeros(size(x));
v_x =  zeros(size(x));

for i=1:length(x)
   time_idx = find(time>=x(i),1,'first');
   time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
   accel_x(i) =lagrange(x(i),time(time_vec),accel(time_vec));
   comp_x(i) =lagrange(x(i),time(time_vec),heading_comp(time_vec));
   v_x(i) = lagrange(x(i),time(time_vec),v_real(time_vec));
end

f_cable_frameBoat = f_cable;% Transform the force of the cable to the frame of the boat
for i=1:length(f_cable_frameBoat(:,1))
   f_cable_frameBoat(i,:)=([cos(-comp_x(i)) -sin(-comp_x(i)) 0;
        sin(-comp_x(i)) cos(-comp_x(i)) 0;
        0 0 1]*(f_cable(i,:)'))';
    
end


%%
errorLdot = zeros(length(x),rode_number);% compute the error of the simulation on the length of the cable
for j=1:length(x)
    for k=1:rode_number
        errorLdot(j,k) = norm(b(j,1+(k-1)*3:k*3))-L(k);
    end
end


%% visu

%v = VideoWriter('newfile.avi','Uncompressed AVI');
%aviobj = avifile('example_osci.avi','compression','None','fps',25);

draw_cable_ = 0;

ratio = 1;
rod_end  = zeros(length(1:ratio:length(x)-1),3);
rod_end_2 = zeros(length(1:ratio:length(x)-1),3);
v_2 = zeros(length(1:ratio:length(x)-1),1);

v_ = sqrt(y(:,4*3*rode_number+rode_number+4).^2+y(:,4*3*rode_number+rode_number+5).^2);

if draw_cable_
    figure(666)
end

jk = 0;
for i=1:ratio:length(x)-1
    %% cable
    jk = jk+1;
    
    time_idx = find(time>=x(i),1,'first');
    
    time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
    v_v = lagrange(x(i),time(time_vec),v_real(time_vec));
    v_2(jk)=v_v;
    
    pos_b = pos_boat(i,:);
    l = sum(L);
    
    l = pos_boat(i,:);
    reshapeB = reshape(b(i,:),3,rode_number);
    rod_end(jk,:) = sum(reshapeB,2)'-[0,0,0.3];
    rod_end_2(jk,:) = sum(reshapeB(:,1:rode_number/coeff_div_pressure_sensor),2)'-[0,0,0.3];
    for number_body=1:rode_number
        
        point=pos_boat(i,:);
        for j=1:number_body
            point = point+b(i,1+(j-1)*3:j*3);
        end
        l = [l; point];
    end
    
    
    
    if draw_cable_
        clf
        
        axis([-10+pos_b(1) 10+pos_b(1)...
            -10+pos_b(2) 10+pos_b(2)...
            -10+pos_b(3) 2+pos_b(3)])
        axis vis3d
        draw_cable(l,666,['r','g','b']);
        title(sprintf('Time : %.3f',i*stepH));
        drawnow
        %aviobj = addframe(aviobj,gcf);
        pause(ratio*stepH/10)
        
    end
end

%%
%viobj = close(aviobj)

%% analyse

size_buff = 70;
rod_end_n = zeros(1,length(rod_end(:,1)));
rod_end_n_2 = zeros(1,length(rod_end(:,1)));
for i=1:length(rod_end_n)
    rod_end_n(i) = mean(rod_end(max(1,i-size_buff):min(i+size_buff,length(rod_end_n)),3));
    rod_end_n_2(i) = mean(rod_end_2(max(1,i-size_buff):min(i+size_buff,length(rod_end_n_2)),3));
end


size_buff = 100;
f_frame_n = zeros(3,length(rod_end(:,1)));
for i=1:length(rod_end_n)
    f_frame_n(1,i) = mean(f_cable_frameBoat(max(1,i-size_buff):min(i+size_buff,length(rod_end_n)),1));
    f_frame_n(2,i) = mean(f_cable_frameBoat(max(1,i-size_buff):min(i+size_buff,length(rod_end_n_2)),2));
    f_frame_n(3,i) = mean(f_cable_frameBoat(max(1,i-size_buff):min(i+size_buff,length(rod_end_n_2)),3));

end

rho = 1000;
radius=0.005;
ms=sum(m);
g=9.81;
CD=1.2;

time_2 = x(1:ratio:length(x)-1);
v_2_simu =v_2;
L_=length_cable;
depth_comp_2 =-cos(atan(CD*2*radius*L_*rho*v_2_simu.^2/(2*g*(rho*pi*L_*radius^2-ms))))*L_/coeff_div_pressure_sensor-0.3; 

figure
subplot(2,1,1)
hold on
plot(time_2-time_2(1),rod_end_n_2(:))
plot(time_2-time_2(1),rod_end_2(:,3),'c')
plot(time-time(1),press_norm,'g')
plot(time_2-time_2(1),depth_comp_2(:),'r');
hold off
t=title(['Simulation Depth cable over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
subplot(2,1,2)
plot(time_2-time_2(1),v_2)

figure
plot(v_2,rod_end_2(:,3),'x')
