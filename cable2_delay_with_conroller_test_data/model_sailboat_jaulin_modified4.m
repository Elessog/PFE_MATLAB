function [dy,alpha_cable] = model_sailboat_jaulin_modified4(y,a,phi,delta_s,delta_r,force_cable)
theta =y(4);
v=y(5);
v_cable =y(6:7);
omega=y(8);

%Internal constants
p1 = 0.03;          % -         drift coefficientv_norm
p2 = 40;            %kg s^-1    tangential friction
p3 = 6000;          %kg m       angular friction
p4 = 200;          %kg s^-1    sail lift
p5 = 1500;          %kg s^-1    rudder lift
p6 = 0.5;             %m          distance to sail CoE
p7 = 0.5;             %m          distance to mast
p8 = 2;             %m          distance to rudder
p9 = 300;           %kg         mass of boat
p10 = 400;        %kg m^2     mass moment of intertia
p11 = 0.2; %rudder break coefficient

%link equations
W_ap = [a*cos(phi-theta)-v a*sin(phi-theta)];
%apperent wind speed vector in b-frame
phi_ap = atan2(W_ap(2),W_ap(1));    %apperent wind angle in b-frame
a_ap = hypot(W_ap(1),W_ap(2));      %apperent wind speed velocity in b-frame


sigma=cos(phi_ap)+cos(delta_s);
if (sigma<0),
    delta_s=pi+phi_ap; %get the sail controled by the wind
else
    if sin(phi_ap)~=0
        delta_s=-sign(sin(phi_ap))*abs(delta_s);
    end
end;

Fs = p4*a_ap*sin(delta_s-phi_ap);  %Force of wind on sail
Fr = p5*v*sin(delta_r);             %Force of water on rudder

cable_norm = sqrt(force_cable(1)*force_cable(1)+force_cable(2)*force_cable(2));

if cable_norm==0
    alpha_cable = 0;
else
    alpha_cable = atan2(force_cable(2),force_cable(1));
end

x_dot =  v*cos(theta) + v_cable(1)+ p1*a*cos(phi);     %x_dot
y_dot =    v*sin(theta) + v_cable(2) + p1*a*sin(phi);  %y_dot
v_dot =   ((Fs*sin(delta_s)-p11*Fr*sin(delta_r))-sign(v)*(p2*(v)^2)+cable_norm*cos(alpha_cable-theta))/p9;
%differential equations
angle_v = atan2(v_cable(2),v_cable(1));
v_dot_cable = (-(p2+3000*sin(angle_v-theta)^2)*(v_cable.*abs(v_cable))+cable_norm*sin(alpha_cable-theta)*[cos(theta+pi/2);sin(theta+pi/2)])/p9;

theta_dot = omega;                %theta_dot
omega_dot = (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r)...
    -p3*omega*norm(v)...
    -(p8+1)*cable_norm*sin(alpha_cable-theta))/p10; %omega_dot
%*cos(delta_r)
dy = [x_dot,y_dot,0,theta_dot,v_dot,v_dot_cable(1),v_dot_cable(2),omega_dot]';

end