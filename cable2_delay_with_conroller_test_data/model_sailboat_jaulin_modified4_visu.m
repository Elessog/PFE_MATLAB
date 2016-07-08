function [force_v_dot,v_dot_main,omega_dot,v_dot_cable,f_rudder,f_sail,f_frict] = model_sailboat_jaulin_modified4_visu(y,a,phi,delta_s,delta_r,force_cable)

theta =y(4);
v=y(5);
v_cable=y(6:7);
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


angle_v = atan2(v_cable(2),v_cable(1));
adjusted_v = v+norm(v_cable)*cos(theta-angle_v);


%link equations
W_ap = [a*cos(phi-theta)-adjusted_v a*sin(phi-theta)];
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
Fr = p5*adjusted_v*sin(delta_r);             %Force of water on rudder

cable_norm = sqrt(force_cable(1)*force_cable(1)+force_cable(2)*force_cable(2));

if cable_norm==0
    alpha_cable = 0;
else
    alpha_cable = atan2(force_cable(2),force_cable(1));
end

force_v_dot =cable_norm*cos(alpha_cable-theta)*[cos(theta),sin(theta)]/p9;
v_dot_main =    (((Fs*sin(delta_s)-p11*Fr*sin(delta_r))-sign(adjusted_v)*(p2*(adjusted_v)^2)+cable_norm*cos(alpha_cable-theta))*[cos(theta),sin(theta)])/p9;
f_sail=Fs*sin(delta_s)*[cos(theta),sin(theta)]/p9;
f_rudder = -p11*Fr*sin(delta_r)*[cos(theta),sin(theta)]/p9;
f_frict = -sign(adjusted_v)*(p2*(adjusted_v)^2)*[cos(theta),sin(theta)]/p9;
x_dot =  v*cos(theta) + v_cable(1)+ p1*a*cos(phi);     %x_dot
y_dot =    v*sin(theta) + v_cable(2) + p1*a*sin(phi);  %y_dot
angle_v = atan2(v_cable(2),v_cable(1));
v_dot_cable = (-(p2+3000*sin(angle_v-theta)^2)*(v_cable.*abs(v_cable))+cable_norm*sin(alpha_cable-theta)*[cos(theta+pi/2),sin(theta+pi/2)])/p9';
omega_dot = [Fs*(p6-p7*cos(delta_s)),-p8*Fr*cos(delta_r),-p3*omega*norm(adjusted_v),-(p8+1)*cable_norm*sin(alpha_cable-theta)]/p10; %omega_dot
%differential equations *cos(delta_r)

end