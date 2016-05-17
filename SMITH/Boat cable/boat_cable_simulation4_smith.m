function [dy,force_cable] = boat_cable_simulation4_smith(t,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%postion  data in y
%y = [b r , bdot ,rdot,errorint,x,y,z,theta,x_dot,y_dot,theta_dot]


%% Cable
global  Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kdl Kpl...
    L vect_z boat_s_dot boat_s_dotdot max_windspeed ...
    time_const_wind psi Lg mg delta_r_s delta_s_s  ... ;



radius = 0.005;

b = y(1:3*rode_number);
r = y(3*rode_number+1:2*3*rode_number);
bdot = y(2*3*rode_number+1:3*3*rode_number);
rdot = y(3*3*rode_number+1:4*3*rode_number);
errorint = y(4*3*rode_number+1:4*3*rode_number+rode_number);
boat_pos = y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);
y_boat = y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+8);

%boat
N = Nn1c;
Ndot = Nn1c;
Ndotdot =Nn1c;

N(1:3,1) =boat_pos;
Ndot(1:3,1) = boat_s_dot;
Ndotdot(1:3,1) =boat_s_dotdot;



%cable
Ldot = zeros(rode_number,1);
%fa and fb construction
P_Arch = ((-mg*9.81+1000*9.81*pi*Lg*radius^2)/2).*vect_z;
FluidFriction = (-1*2*radius*Lg*1000/2).*rdot;


fa=zeros(3*rode_number,1)+P_Arch+FluidFriction;
fb=fa;

tau1 = (fa+fb)./mg;

C = Wn1c*r+Pn1c*b+N;

Cdot = Wn1c*rdot+Pn1c*bdot+Ndot;

%calc fa fb orthog to the rods for each rods
fap=fa;
fbp=fb;
for i = 1:rode_number
    fbi=fb(1+(i-1)*3:i*3,:);%fa' and fb' orthogonal to b
    fai=fa(1+(i-1)*3:i*3,:);
    bn =b(1+(i-1)*3:i*3,:);
    fap(1+(i-1)*3:i*3,:) = fai-bn*(fai'*bn)/(b'*b);
    fbp(1+(i-1)*3:i*3,:) = fai-bn*(fbi'*bn)/(b'*b);
end

[ tau2,lambda ] = bar_length_control(fap,fbp,b,bdot,errorint,Ldot,tau1,...
    Ndotdot,Cdot,C,Kdl,Kpl);

tauc1 = -Wn1c*lambda;

tauc2 = 6*(Wn1ca-Wn1cb)*lambda;

rdotdot=tau1+tauc1;
bdotdot=tau2+tauc2;


errorLdot = errorint;
for k=1:rode_number
    errorLdot(k) = norm(b(1+(k-1)*3:k*3))-L(k);
end


dy = vertcat(bdot,rdot);
dy = vertcat(dy,bdotdot);
dy = vertcat(dy,rdotdot);
dy = vertcat(dy,errorLdot);
%% compute force cable
tauc1r = reshape(mg.*tauc1,3,rode_number);
force_cable =sum(tauc1r,2);
%% Boat

windspeed = max_windspeed*(1-exp(-t/time_const_wind));

% [a,b,index_out] = path_planning_v_control(y_boat(1),y_boat(2),index_out);
% 
% if mod(t,1/controller_freq)<(1/controller_freq)*size_rect_cont
%      [delta_r_s, delta_s_s] = controller_simpleLine_v_control(y_boat(1),...
%          y_boat(2), y_boat(4), y_boat(5),psi, a, b);
% end


[dy_boat,alpha_cable] = model_sailboat_jaulin_modified4(y_boat,windspeed,...
    psi,delta_s_s,delta_r_s,-force_cable);

boat_s_dot = [dy_boat(1:2);0];
boat_s_dotdot = [dy_boat(5)*cos(y_boat(4))+dy_boat(6);...
    dy_boat(5)*sin(y_boat(4))+dy_boat(7);0];


dy = vertcat(dy,dy_boat);
%% Smith predictor




end

