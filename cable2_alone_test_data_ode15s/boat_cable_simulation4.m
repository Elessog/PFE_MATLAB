function dy= boat_cable_simulation4(t,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%postion  data in y
%y = [b r , bdot ,rdot,errorint,x,y,z,theta,x_dot,y_dot,theta_dot]


%% Cable
global  Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kdl Kpl...
    L  vect_z  boat_pos boat_dot boat_dotdot Lg mg accel time east_north_real...
    heading_comp v_real coeff_div_pressure_sensor boolPrint coefSpring coefDotSpring;


%% readjustment of position
%get time index in data
time_idx = find(time>=t,1,'first');

if mod(t,1)<0.1
    if boolPrint
        boolPrint= 0;
        fprintf('Time : %.5f / %.3f\n',t,time(end));
    end
else
    boolPrint = 1;
end

time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
east_x = lagrange(t,time(time_vec),east_north_real(1,time_vec));
east_y = lagrange(t,time(time_vec),east_north_real(2,time_vec));

v_v = lagrange(t,time(time_vec),v_real(time_vec));
v_acc = lagrange(t,time(time_vec),accel(time_vec));
v_comp =lagrange(t,time(time_vec),heading_comp(time_vec));
boat_pos = [east_x;east_y;-0.30];

boat_dot = [v_v*cos(v_comp);v_v*sin(v_comp);0];
boat_dotdot =[v_acc*cos(v_comp);v_acc*sin(v_comp);0];
y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3) = boat_pos;
y(4*3*rode_number+rode_number+4:4*3*rode_number+rode_number+6) = boat_dot;


radius = 0.005;

b = y(1:3*rode_number);
r = y(3*rode_number+1:2*3*rode_number);
bdot = y(2*3*rode_number+1:3*3*rode_number);
rdot = y(3*3*rode_number+1:4*3*rode_number);
errorint = y(4*3*rode_number+1:4*3*rode_number+rode_number);
%boat_pos = y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);

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
P_Arch = ((-mg*9.81+1000*9.81*pi*Lg*radius^2)).*vect_z;

%FluidFriction = (-1.2*2*radius*Lg*1000/2).*rdot.*abs(rdot);
Kfluid = (1.2*2*radius*1000/2);
pressure_sensor_pos=(rode_number/coeff_div_pressure_sensor-1)*3+1:(rode_number/coeff_div_pressure_sensor-1)*3+3;
pressure_sensor_cable = 1:pressure_sensor_pos(end);

% %adding effect of the pressure sensor
P_Arch(pressure_sensor_pos) = P_Arch(pressure_sensor_pos) + (1000*9.81*pi*0.133*0.024^2).*vect_z(pressure_sensor_pos);
% FluidFriction(pressure_sensor_pos) = FluidFriction(pressure_sensor_pos) +...
%     (-1.2*0.024*0.133*1000/2).*rdot(pressure_sensor_pos).*abs(rdot(pressure_sensor_pos));
% 
% %adding effect of pressure sensor cable
% FluidFriction(pressure_sensor_cable) = FluidFriction(pressure_sensor_cable) +...
%     (-1.2*0.01*abs(b(pressure_sensor_cable))*1000/2).*rdot(pressure_sensor_cable).*abs(rdot(pressure_sensor_cable));
% 
% 


fa=P_Arch/2;%+FluidFriction;
fb=fa;

%%compute spring
% 
 for index=1:rode_number
%     
%     if index~=0
%         b1 = b(1+(index-1)*3:3*index);
%         b1dot = bdot(1+(index-1)*3:3*index);
%     else
%         b1 = -boat_dot;
%         b1dot = -boat_dotdot;
%     end
%     b2 = b(1+(index+1-1)*3:3*(index+1));
%     b2dot = bdot(1+(index+1-1)*3:3*(index+1));
%     
%     valueNorm = dot(b1,b2)/(norm(b1)*norm(b2));
%     if abs(valueNorm)>1
%         %       %fprintf('go over 1 norm b1 %0.2f norm b2 %0.2f index %1.0f\n',norm(b1),norm(b2),index);
%         valueNorm=1;
%     end
%     
%     angle = wrapToPi(acos(valueNorm));
%     NormFres =-coefSpring*angle;
%     unitVect = cross(b1,b2);
%     if abs(sin(angle))>=1e-7
%         F2vect = cross(b2,unitVect);
%         F2vectVal = NormFres*F2vect/norm(F2vect);
%         F1vect = cross(b1,unitVect);
%         F1vectVal = NormFres*F1vect/norm(F1vect);
%         
%         if index>0
%         fa(1+(index-1)*3:3*index) = fa(1+(index-1)*3:3*index) +F1vectVal;
%         end
%         fb(1+(index+1-1)*3:3*(index+1)) = fb(1+(index+1-1)*3:3*(index+1)) +F2vectVal;
%         %% damping term
%         angle_dot = (dot(b1dot,b1)+dot(b2dot,b2)-(dot(b1dot,b2)+dot(b1,b2dot)))/(norm(b1)*norm(b2)*sin(angle));
%         NormFfr= -sign(angle_dot)*coefDotSpring*angle_dot.^2;
%         F2_vect = F2vect/norm(F2vect);
%         F1_vect = F1vect/norm(F1vect);
% 
%         if index>0
%         fa(1+(index-1)*3:3*index) = fa(1+(index-1)*3:3*index) + F1_vect*NormFfr;
%         end
%         fb(1+(index+1-1)*3:3*(index+1)) = fb(1+(index+1-1)*3:3*(index+1)) + F2_vect*NormFfr;
%     end
%     
    b1 = b(1+(index-1)*3:3*index);
    omega = bdot(1+(index-1)*3:3*index)/norm(b1);
    r1dot = rdot(1+(index-1)*3:3*index);
    crossB_omega = cross(b1,omega);
%     base_speed_vector = (norm(b1)/2)*r1dot.^2+(norm(b1)/2)*crossB_omega.^2;
%     speed_vector_a = base_speed_vector-(norm(b1))*r1dot.*crossB_omega;
%     speed_vector_b = base_speed_vector+(norm(b1))*r1dot.*crossB_omega;
     speed_sign_a = sign(r1dot-crossB_omega);
     speed_sign_b = sign(r1dot+crossB_omega);
     base_speed_vector =(norm(b1)/2)*r1dot.^2+(norm(b1)/2)*crossB_omega.^2;
     if rode_number ==1
        speed_vector_a = base_speed_vector+(norm(b1)/3)*r1dot.*crossB_omega;%+(norm(b1)/2)*crossB_omega.^2;
         speed_vector_b = base_speed_vector+(norm(b1)/3)*r1dot.*crossB_omega;%+(norm(b1)/2)*crossB_omega.^2; 
     else
         speed_vector_a = base_speed_vector-(norm(b1)/3)*r1dot.*crossB_omega;
         speed_vector_b = base_speed_vector+(norm(b1)/3)*r1dot.*crossB_omega;
     end
         
     fa(1+(index-1)*3:3*index) = fa(1+(index-1)*3:3*index) - speed_sign_a.*Kfluid.*speed_vector_a;
     fb(1+(index-1)*3:3*index) = fb(1+(index-1)*3:3*index) - speed_sign_b.*Kfluid.*speed_vector_b;
     
end





tau1 = (fa+fb)./(mg);

C = Wn1c*r+Pn1c*b+N;

Cdot = Wn1c*rdot+Pn1c*bdot+Ndot;

%calc fa fb orthog to the rods for each rods
fap=fa;
fbp=fb;
for i = 1:rode_number
    fbi=fb(1+(i-1)*3:i*3,:);%fa' and fb' orthogonal to b
    fai= fa(1+(i-1)*3:i*3,:);
    bn =b(1+(i-1)*3:i*3,:);
    fap(1+(i-1)*3:i*3,:) = fai-bn*(fai'*bn)/(b'*b);
    fbp(1+(i-1)*3:i*3,:) = fbi-bn*(fbi'*bn)/(b'*b);
end

[ tau2,lambda ] = bar_length_control(fap,fbp,b,bdot,errorint,Ldot,tau1,Ndotdot,Cdot,C,Kdl,Kpl);

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
dy = vertcat(dy,boat_dot);
dy = vertcat(dy,boat_dotdot);
%% compute force cable
%tauc1r = reshape(mg.*tauc1,3,rode_number);
%force_cable =sum(tauc1r,2);
% if norm(force_cable)>1000
%     force_cable = 1000*force_cable/norm(force_cable);
% end
end

