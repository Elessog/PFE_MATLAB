function [delta_r, delta_sMax] = controller_simpleLine_v_control(x,y,theta,v_boat, psi, a, b)
global q v_target pos_sum active_os tacking
%controller_simpleLine Simple line following controller
%   Controller based on the paper "A simple controller for line 
%   following of sailboats" by Luc Jaulin and Fabrice Le Bars
%Constans/Parameters
m = [x,y];
if active_os
  m = m + pos_sum(1:2);
  theta = theta+pos_sum(3);
end
r = 5;             % m -   cutoff distance
delta_rMax = pi/4;  % rad   maximum rudder angle
gamma = pi/4;       % rad   incidence angle
xi = pi/3;          % rad   close hauled angle
xi_tack = pi/4;     % rad   tack heading

%Step 3
phi = atan2((b(2)-a(2)),(b(1)-a(1)));

%Step 1
u = 1./hypot(b(1)-a(1),b(2)-a(2)).*[b(1)-a(1) b(2)-a(2)];
v = [m(1)-a(1) m(2)-a(2)];
e = u(1)*v(2)-v(1)*u(2);

% %Step 2
if (abs(e) > r)
    q = sign(e);
end

%Step 4
theta_star = phi-2*gamma/pi*atan(e/r);

%Step 5-9
if ((cos(psi-theta_star)+cos(xi) < 0) || ...
        ((abs(e)<r)&&(cos(psi-phi)+cos(xi)<0)))
    if ~tacking
        v2 = [m(1)+100*cos(theta+pi)-a(1) m(2)+100*sin(theta+pi)-a(2)];
        e2 = u(1)*v2(2)-v2(1)*u(2);
        q = sign(e2);
        tacking = 1;
    end
    theta_bar = pi + psi - q*xi;
% elseif (v_target-v_boat)<0
%     theta_bar = theta_star-sign(e)*xi*exp(abs(v_target-v_boat));
else
    tacking = 0;
    theta_bar = theta_star;
end

%Step 10-11
if cos(theta-theta_bar) >= 0
    delta_r = delta_rMax*sin(theta-theta_bar);
else
    delta_r = delta_rMax*sign(sin(theta-theta_bar));
end

%Step 12
delta_sMax = pi/4*(cos(psi-theta_bar)+1);

end