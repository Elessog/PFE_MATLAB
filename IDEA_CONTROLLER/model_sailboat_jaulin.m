function dy = model_sailboat_jaulin(y,a,phi,delta_s,delta_r)
global v_dot fs phi_ap W_ap;
theta =y(4);
v=y(5);
omega=y(6);

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
    delta_s=pi+phi_ap;
else
    if sin(phi_ap)~=0
        delta_s=-sign(sin(phi_ap))*delta_s;
    end
end;

Fs = p4*a_ap*sin(delta_s-phi_ap);  %Force of wind on sail
Fr = p5*v*sin(delta_r);             %Force of water on rudder

x_dot =  v*cos(theta) + p1*a*cos(phi);     %x_dot
y_dot =    v*sin(theta) + p1*a*sin(phi);  %y_dot
fs = Fs*sin(delta_s);
v_dot =   ((Fs*sin(delta_s)-p11*Fr*sin(delta_r))-sign(v)*(p2*(v)^2))/p9;
theta_dot = omega;                %theta_dot
omega_dot = (Fs*(p6-p7*cos(delta_s))-p8*Fr*cos(delta_r)...
    -p3*omega*norm(v)...
     )/p10; %omega_dot
%*cos(delta_r)
dy = [x_dot,y_dot,0,theta_dot,v_dot,omega_dot]';

end