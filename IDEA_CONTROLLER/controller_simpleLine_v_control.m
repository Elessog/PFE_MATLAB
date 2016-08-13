function [delta_r, delta_sMax] = controller_simpleLine_v_control(x,y,theta,v_boat, psi, a, b,t)
global q depth_target m tacking L coeff_rudder_speed
global old_t old_diff_v old_diff_v_dot diff_v_dot coeff_d_rudder_speed
%controller_simpleLine Simple line following controller
%   Controller based on the paper "A simple controller for line
%   following of sailboats" by Luc Jaulin and Fabrice Le Bars
%Constans/Parameters


rho = 1000;
radius=0.005;
ms=sum(m);
g=9.81;
CD=1.2;

L_=sum(L);
v_target = sqrt((2*g*(ms-rho*pi*L_*radius^2))*tan(acos(-(depth_target +0.3 )/L_))/(CD*2*radius*L_*rho));

m_pos = [x,y];
% if active_os
%   m_pos = m_pos + pos_sum(1:2);
%   theta = theta+pos_sum(3);
% end
r = 5;             % m -   cutoff distance
delta_rMax = pi/4;  % rad   maximum rudder angle
gamma = pi/4;       % rad   incidence angle
xi = pi/3;          % rad   close hauled angle
xi_tack = pi/4;     % rad   tack heading

%Step 3
phi = atan2((b(2)-a(2)),(b(1)-a(1)));

%Step 1
u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
v = [m_pos(1)-a(1) m_pos(2)-a(2)];
e = u(1)*v(2)-v(1)*u(2);

% %Step 2
if (abs(e) > r)
    q = sign(e);
end

%Step 4
theta_star = phi-2*gamma/pi*atan(e/r);

diff_v= v_target-v_boat;

if t-old_t~=0
  diff_v_dot = (diff_v-old_diff_v)/(t-old_t);
  old_diff_v_dot = diff_v_dot;
end

old_diff_v = diff_v;
old_t = t;

 %Step 5-9
if ((cos(psi-theta_star)+cos(xi) < 0) || ...
        ((abs(e)<r)&&(cos(psi-phi)+cos(xi)<0)))
    if ~tacking
        v2 = [m_pos(1)+100*cos(theta+pi)-a(1) m_pos(2)+100*sin(theta+pi)-a(2)];
        e2 = u(1)*v2(2)-v2(1)*u(2);
        q = sign(e2);
        tacking = 1;
    end
    theta_bar = pi + psi - q*xi;
else
    tacking = 0;
    theta_bar = theta_star;
end


%Step 10-11
if cos(theta-theta_bar) >= 0
    delta_r = delta_rMax*sin(theta-theta_bar);
%     if (diff_v)<0
%         delta_r_o = delta_r-sign(e)*pi/2*(coeff_rudder_speed*abs(diff_v)+coeff_d_rudder_speed*abs(diff_v_dot));
%         
%         fprintf('v_target %.2f  v %.2f delta_r : %.2f %.2f\n',v_target,v_boat,delta_r_o,delta_r);
%         delta_r = delta_r_o;
%     end
else
    delta_r = delta_rMax*sign(sin(theta-theta_bar));
end

%Step 12
delta_sMax = pi/4*(cos(psi-theta_bar)+1);

if diff_v<=0
    delta_sMax =0;% delta_sMax/exp(10*abs(diff_v));
elseif v_boat>0
    delta_sMax_o = delta_sMax/exp((v_target/abs(diff_v)-1)/20);
    fprintf('v_target %.2f  v %.2f delta_s : %.2f %.2f\n',v_target,v_boat,delta_sMax_o,delta_sMax);
    delta_sMax = delta_sMax_o;
%         
end





end