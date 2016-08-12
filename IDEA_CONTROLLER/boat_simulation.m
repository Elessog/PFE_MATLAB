function dy = boat_simulation(t,y)

global index_out max_windspeed ...
    time_const_wind psi delta_r delta_s ...
    controller_freq size_rect_cont control_computed ...
    coeff_div_pressure_sensor boolPrint ...
    boat_dot boat_dotdot  Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kdl Kpl...
    L vect_z Lg mg;
    

if mod(t,1)<0.1
    if boolPrint
        boolPrint= 0;
        fprintf('Time : %.5f \n',t);
    end
else
    boolPrint = 1;
end

radius = 0.005;

b = y(1:3*rode_number);
r = y(3*rode_number+1:2*3*rode_number);
bdot = y(2*3*rode_number+1:3*3*rode_number);
rdot = y(3*3*rode_number+1:4*3*rode_number);
errorint = y(4*3*rode_number+1:4*3*rode_number+rode_number);
boat_pos = y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);
y_boat = y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+8);



%% update  boat position for cable simulation
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

%% adding effect of the pressure sensor
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
fap=fa;
fbp=fb;

for index=1:rode_number
    %% compute fluid friction
    b1 = b(1+(index-1)*3:3*index);
    omega = bdot(1+(index-1)*3:3*index)/norm(b1);
    r1dot = rdot(1+(index-1)*3:3*index);
    crossB_omega = cross(b1,omega);
    speed_sign_a = sign(r1dot-crossB_omega);
    speed_sign_b = sign(r1dot+crossB_omega);
    base_speed_vector =(norm(b1)/2)*r1dot.^2+(norm(b1)/2)*crossB_omega.^2;
    if rode_number ==1
        speed_vector_a = base_speed_vector-(norm(b1)/3)*r1dot.*crossB_omega;%+(norm(b1)/2)*crossB_omega.^2;
        speed_vector_b = base_speed_vector+(norm(b1)/3)*r1dot.*crossB_omega;%+(norm(b1)/2)*crossB_omega.^2;
    else
        speed_vector_a = base_speed_vector-(norm(b1)/3)*r1dot.*crossB_omega;
        speed_vector_b = base_speed_vector+(norm(b1)/3)*r1dot.*crossB_omega;
    end
    
    fa(1+(index-1)*3:3*index) = fa(1+(index-1)*3:3*index) - speed_sign_a.*Kfluid.*speed_vector_a;
    fb(1+(index-1)*3:3*index) = fb(1+(index-1)*3:3*index) - speed_sign_b.*Kfluid.*speed_vector_b;
    
    %% calc fa fb orthog to the rods for each rods
    fbi=fb(1+(index-1)*3:index*3,:);%fa' and fb' orthogonal to b
    fai= fa(1+(index-1)*3:index*3,:);
    bn =b(1+(index-1)*3:index*3,:);
    fap(1+(index-1)*3:index*3,:) = fai-bn*(fai'*bn)/(b'*b);
    fbp(1+(index-1)*3:index*3,:) = fbi-bn*(fbi'*bn)/(b'*b);
end


tau1 = (fa+fb)./(mg);

C = Wn1c*r+Pn1c*b+N;

Cdot = Wn1c*rdot+Pn1c*bdot+Ndot;

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
%% compute force cable
tauc1r = reshape(mg.*tauc1,3,rode_number);
force_cable =sum(tauc1r,2);
% if norm(force_cable)>1000
%     force_cable = 1000*force_cable/norm(force_cable);
% end
%% Boat

windspeed = max_windspeed*(1-exp(-t/time_const_wind));

[a,b,index_out] = path_planning_v_control(y_boat(1),y_boat(2),index_out);


%% frequency of actuators
if mod(t,1/controller_freq)<(1/controller_freq)*size_rect_cont
   if (~control_computed)
     [delta_r, delta_s] = controller_simpleLine_v_control(y_boat(1),...
         y_boat(2), y_boat(4),norm(boat_dot),psi, a, b,t);
     control_computed = 1;
   end
else
    control_computed = 0;
end

dy_boat= model_sailboat_jaulin_modified4(y_boat,windspeed,...
    psi,delta_s,delta_r,-force_cable);

boat_dot = [dy_boat(1:2);0];
boat_dotdot = [dy_boat(5)*cos(y_boat(4))+dy_boat(6);...
    dy_boat(5)*sin(y_boat(4))+dy_boat(7);0];


dy = vertcat(dy,dy_boat);
dy = vertcat(dy,boat_dotdot);

end

