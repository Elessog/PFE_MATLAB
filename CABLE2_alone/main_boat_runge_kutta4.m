close all;clc;
global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_dot boat_dotdot coeff_div_pressure_sensor;
global L Lg mg m boolPrint coefSpring speed_g;

rode_number = 4;%number of rods to simulate the cable
coeff_div_pressure_sensor = 2; %divisor for placing the pressure sensor on the cable (as integer)
% 2 will mean on rod 2 if rod_number = 4 or 1 if rod number = 3
length_cable = 6;
coefSpring =0.5;%2;
coefDotSpring = -18;%-10;
boolPrint = 1;

mass_vect = 0.1:0.1:0.9;
angle_cable = zeros(length(mass_vect),1);
force_end_cable = zeros(length(mass_vect),3);
index = 0;
vect_speed = 0.01:0.02:3;

%speed = 1;


end_force = [];
end_depth = [];
speed = 2;

for mass_lin =  mass_vect
    
%% initialization of the state of the boat
index = index+1
origin = [0;0;0];
boat_dot=[speed;0;0];
boat_dotdot=[0;0;0];
theta_0 = 0;
speed_g = speed;
%% Matrix creation for the cable simulation
%b initial vector of the rods
% b_0 = [0 0 0
%     0  0 0
%     -1 -1 -1];
L=4*ones(rode_number,1);
Lg = 4*ones(rode_number*3,1);
Lg(rode_number*3-2:rode_number*3) = 4;
b_0 = zeros(3,rode_number);
% b_0(1,:)=-L'.*cos(theta_0).*ones(1,rode_number);
% b_0(2,:)=-L'.*sin(theta_0).*ones(1,rode_number);
b_0(3,:)=-L'.*ones(1,rode_number);
dl = mass_lin;
m = dl*L;
mg = dl*Lg;
r_0 = zeros(3,rode_number);
r_0(:,1) = b_0(:,1)/2.0;

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
wnl=1000;
zeta=1;
Kpl=wnl^2;
Kdl=2*zeta*wnl;
Kil=10000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lambdainverse = ((Wn1c*Wn1c'-6*Pn1c*(Wn1ca'-Wn1cb'))^-1);%unchanging part of the lagrangian multipliers calculation

q1dot = ones(size(q1)).*boat_dotdot(1).*vect_x;

%% creation vector y for the differential solving
y0 = vertcat(q2,q1);
y0 = vertcat(y0,zeros(size(q2)));
y0 = vertcat(y0,zeros(size(q1)));
y0 = vertcat(y0,zeros(rode_number,1));
y0 = vertcat(y0,[origin;0;0;0]);

%%%%%% Time parameters %%%%%%%
stepH = 0.1;
x= 0:stepH:50;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire


%% computation of the ODE
options = odeset('RelTol',1e-1,'AbsTol',1e-3);

[t,y] = ode45(@boat_cable_simulation4,x,y0,options);

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
  fa=(P_Arch+FluidFriction')/2;
  fb=fa;
  tau1 = (fa+fb)./mg;
  tauc1 = rdotdot_(i-1,:)'-tau1;
  tauc1r = reshape(mg.*tauc1,3,rode_number);
  f_cable(i,:) =sum(tauc1r,2)';
end


f_cable_frameBoat = f_cable;% Transform the force of the cable to the frame of the boat
for i=1:length(f_cable_frameBoat(:,1))
   f_cable_frameBoat(i,:)=([cos(-theta_0) -sin(-theta_0) 0;
        sin(-theta_0) cos(-theta_0) 0;
        0 0 1]*(f_cable(i,:)'))';
    
end


%%
errorLdot = zeros(length(x),rode_number);% compute the error of the simulation on the length of the cable
for j=1:length(x)
    for k=1:rode_number
        errorLdot(j,k) = norm(b(j,1+(k-1)*3:k*3))-L(k);
    end
end

end_force = [end_force;f_cable_frameBoat(end,:)];
end_depth = [end_depth sum(reshape(b(end,:),3,rode_number),2)];

end
