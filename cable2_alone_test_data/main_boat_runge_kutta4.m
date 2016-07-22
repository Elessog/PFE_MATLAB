clear all;close all;clc;


[FileName,PathName] = uigetfile('*.mat','Select the MATLAB run');

load([PathName,FileName]);

labview_wayponts = 0;
if labview_wayponts
    [FileName2,PathName2] = uigetfile('*.mat','Select the labview waypoints');
    load([PathName2,FileName2]);
    waypoints = [utm_x-origin(1) utm_y-origin(2) (1:length(utm_x))'];
    
    
end

[way_lat,way_lont] = utm2ll(waypoints(:,1)+origin(1),waypoints(:,2)+origin(2),34);
[pos_lat,pos_lont] = utm2ll(east_north(1,:)+origin(1),east_north(2,:)+origin(2),34);

global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_pos boat_dot boat_dotdot;
global L Lg mg m;

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
stepH = 0.001;
x= time(1):stepH:time(end-1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire

f_cable= zeros(3,length(x));

%% computation of the ODE
for i=1:1:(length(x)-2)                           % calculation loop
    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy((x(i)+0.5*stepH),(y(:,i)+0.5*stepH*k_2));
    [k_4,force_cable] = F_xy((x(i)+stepH),(y(:,i)+k_3*stepH));
    f_cable(:,i+1)=force_cable;
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
    %% readjustment of position
    %get time index in data
    time_idx = find(time>=x(i+1),1,'first');
    if(mod(i,1000)==0)
        fprintf('Time: %d/%d index: %d   %d\n',time(time_idx)-time(1),time(end-1)-time(1),time_idx,i);
    end
    
    time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
    dt_temp = (time(time_idx)-x(i+1))/(time(time_idx)-time(time_idx-1));
    east_x = lagrange(x(i+1),time(time_vec),east_north(1,time_vec));
    east_y = lagrange(x(i+1),time(time_vec),east_north(2,time_vec));
    
    v_v = lagrange(x(i+1),time(time_vec),v_real(time_vec));
    v_acc = lagrange(x(i+1),time(time_vec),accel(time_vec));
    v_comp =lagrange(x(i+1),time(time_vec),accel(time_vec));
    boat_pos = [east_x;east_y;0];
    
    boat_dot = [v_v*cos(v_comp);v_v*sin(v_comp);0];
    boat_dotdot =[v_acc*cos(v_comp);v_acc*sin(v_comp);0];
    y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3,i+1) = boat_pos;
    y(4*3*rode_number+rode_number+4:4*3*rode_number+rode_number+6,i+1) = boat_dot;
end

f_cable=f_cable';
%force_end_cable(index,:) = f_cable(30001,:);
%%
y= y';


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


draw_cable = 0;

rod_end  = zeros(length(1:100:length(x)-1),3);

if draw_cable
    figure(666)
end
jk = 0;
ratio = 40;
for i=1:ratio:length(x)-1
    %% cable
    jk = jk+1;
    
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
    
    
    if draw_cable
        clf
        
        axis([-l+pos_b(1) l+pos_b(1)...
            -l+pos_b(2) l+pos_b(2)...
            -l+pos_b(3) 2+pos_b(3)])
        axis vis3d
        draw_cable(l,666,['r','g','b'])
        title(sprintf('Time : %.3f',i*stepH));
        drawnow
        %aviobj = addframe(aviobj,gcf);
        pause(ratio*stepH/10)
        
    end
end


%viobj = close(aviobj)


size_buff = 100;
rod_end_n = zeros(1,length(rod_end(:,1)));
for i=1:length(rod_end_n)
    rod_end_n(i) = mean(rod_end(max(1,i-size_buff):min(i+size_buff,length(rod_end_n)),3));
end

time_2 = x(1:ratio:length(x)-1);