function [dy,force_cable] = boat_cable_simulation4(t,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%postion  data in y
%y = [b r , bdot ,rdot,errorint,x,y,z,theta,x_dot,y_dot,theta_dot]


%% Cable
global  Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kdl Kpl...
    L vect_z boat_dot boat_dotdot windspeed_t...
    psi Lg mg delta_r delta_s delay command_buffer_size ...
    controller_freq size_rect_cont control_computed buffer_command  ...
    idx_bfc delta_s_s delta_r_s waypoints;



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
Ndot(1:3,1) = boat_dot;
Ndotdot(1:3,1) =boat_dotdot;



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
% if norm(force_cable)>1000
%     force_cable = 1000*force_cable/norm(force_cable);
% end
%force_cable = [0,0,0];
%% Boat

windspeed = windspeed_t;


if mod(t,1/controller_freq)<(1/controller_freq)*size_rect_cont
   if (~control_computed)
     control_computed = 1;
     [delta_r,delta_s] = controller_waypoint_v_control(y_boat(1),y_boat(2),y_boat(4),y_boat(5), psi,windspeed, waypoints);
     buffer_command(:,idx_bfc) = [delta_r;delta_s;t];
     idx_bfc = idx_bfc+1;
     
   end
else
    control_computed = 0;
end

if (t>=buffer_command(3,1)+delay)
    delta_r_s = buffer_command(1,1);
    delta_s_s = buffer_command(2,1);
    buffer_command(:,1:command_buffer_size-1) = buffer_command(:,2:command_buffer_size);
    idx_bfc = idx_bfc-1;
    if (idx_bfc==1)
        buffer_command(3,1) = t+1/controller_freq;
    end
end

[dy_boat,alpha_cable] = model_sailboat_jaulin_modified4(y_boat,windspeed,...
    psi,delta_s_s,delta_r_s,-force_cable);

boat_dot = [dy_boat(1:2);0];
boat_dotdot = [dy_boat(5)*cos(y_boat(4))+dy_boat(6);...
    dy_boat(5)*sin(y_boat(4))+dy_boat(7);0];


dy = vertcat(dy,dy_boat);

end

