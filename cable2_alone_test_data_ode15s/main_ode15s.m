clear all;close all;clc;

global east_north;

[FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');

load([PathName,FileName]);


[way_lat,way_lont] = utm2ll(waypoints(:,1)+origin(1),waypoints(:,2)+origin(2),34);
[pos_lat,pos_lont] = utm2ll(east_north(1,:)+origin(1),east_north(2,:)+origin(2),34);

global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_pos boat_dot boat_dotdot;
global L Lg mg m;
global  accel time heading_comp v_real;
rode_number = 4;%number of rods to simulate the cable


%% preprocessing of test data

heading_comp = heading.*(v>=1)+heading2.*(v<1);
IDX = kmeans([v',heading',(heading2-heading)'],2);
v_real_2=v.*(IDX'==1)-v.*(IDX'~=1);

idx1 = abs(wrapTo2Pi(heading-heading2))>pi/2;
idx2 = abs(wrapToPi(heading-heading2))>pi/2;
idx = idx1 | idx2;
v_real = v.*(idx==0)-v.*(idx==1);
v_real=v;
time = fixtime(time);


size_buff = 5;
v_real = zeros(1,length(v));
for i=1:length(v)
    v_real(i) = mean(v(max(1,i-size_buff):min(i+size_buff,length(v))));
end

v_t = v_real;
v_real = zeros(1,length(v));
for i=1:length(v)
    v_real(i) = mean(v_t(max(1,i-size_buff):min(i+size_buff,length(v))));
end


accel = diff(v_real)./diff(time);


%% initialization of the state of the boat

boat_pos = [east_north(1,1);east_north(2,1);0];
boat_dot=[v_real(1)*cos(heading_comp(1));v_real(1)*sin(heading_comp(1));0];
boat_dotdot=[accel(1)*cos(heading_comp(1));accel(1)*cos(heading_comp(1));0];
%theta_0 = heading_comp;
%% Matrix creation for the cable simulation
%b initial vector of the rods
% b_0 = [0 0 0
%     0  0 0
%     -1 -1 -1];
L=(6/4)*ones(rode_number,1);
%L(rode_number) = 0.1;
Lg = (6/4)*ones(rode_number*3,1);
%Lg(rode_number*3-2:rode_number*3) = 4;
b_0 = zeros(3,rode_number);
%b_0(1,:)=-L'.*cos(theta_0).*ones(1,rode_number);
%b_0(2,:)=-L'.*sin(theta_0).*ones(1,rode_number);
b_0(3,:) = -L'.*ones(1,rode_number);

dl = 0.5;
m = dl*L;%mass of rods
mg = dl*Lg;% mass of every direction
m(length(m)) =m(length(m));
mg(length(mg)-2:length(mg))=m(length(m)) ;
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
wnl=500;
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
x= time(1):stepH:time(end-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire

f_cable= zeros(3,length(x));

%% computation of the ODE
[t,y] = ode15s(@boat_cable_simulation4,x,y0);



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


% f_cable_frameBoat = f_cable;% Transform the force of the cable to the frame of the boat
% for i=1:length(f_cable_frameBoat(:,1))
%     f_cable_frameBoat(i,:)=([cos(-theta_boat(i)) -sin(-theta_boat(i)) 0;
%         sin(-theta_boat(i)) cos(-theta_boat(i)) 0;
%         0 0 1]*(f_cable(i,:)'))';
%     
% end

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
    rod_end(jk,:) = sum(reshape(b(i,:),3,rode_number),2)';
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
        pause(ratio*stepH/1)
        
    end
end


%viobj = close(aviobj)


size_buff = 50;
rod_end_n = zeros(1,length(rod_end(:,1)));
for i=1:length(rod_end_n)
    rod_end_n(i) = mean(rod_end(max(1,i-size_buff):min(i+size_buff,length(rod_end_n)),3));
end

time_2 = x(1:ratio:length(x)-1);

figure
subplot(2,1,1)
plot(time_2,rod_end_n)
title(['Depth cable over time ',FileName])
t=title(['Depth cable over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
subplot(2,1,2)
plot(time_2,v_2)

figure
plot(v_2,rod_end_n,'x')
