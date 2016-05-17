function [delta_r, delta_sMax] = controller_simpleLine(x,y, theta, psi, a, b)
global q
%controller_simpleLine Simple line following controller
%   Controller based on the paper "A simple controller for line 
%   following of sailboats" by Luc Jaulin and Fabrice Le Bars
%Constans/Parameters
m = [x,y];
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
if (abs(e) > r/2)
    q = sign(e);
end
%Step 2 - Decide navigation type: nominal or tack
% coder.extrinsic('wrapToPi'); 
% res1=0
% res2=0
% res3=0
% res1 = wrapToPi(psi-pi);
% res2 = wrapToPi(res1-phi);
% res3 = wrapToPi(res1- q*xi_tack);
% if (abs(res2)<xi) %if true - Tack
%     if abs(e)>r/2
%         if sign(e) ~=0    %tack variabel.
%           q=sign(e);
%         end
%     end
%     theta_bar = res3;
% else
%     %Nominal saling - sail along line. second term makes the line
%     %attractive
%     theta_bar = phi-2*gamma/pi*atan(e/r);
% end


%Step 4
theta_star = phi-2*gamma/pi*atan(e/r);

%Step 5-9
if ((cos(psi-theta_star)+cos(xi) < 0) || ...
        ((abs(e)<r)&&(cos(psi-phi)+cos(xi)<0)))
    theta_bar = pi + psi - q*xi;
else
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
% alpha_cable=0;
% if show
%     figure(668)
%     clf         %clear current figure
%     hold on
%     xlabel('x [m]')
%     ylabel('y [m]')
%     axis square
%     axis_max_l = 100;
%     axis_min = -2;
%     s = axis_max_l*.04;
%     axis([axis_min-s axis_min+axis_max_l+s axis_min-s axis_min+axis_max_l+s]);
% 
% 
%     %draw wind direction
%     m_x = axis_min+axis_max_l/2;
%     m_y = axis_min+axis_max_l/2;
%     x_w = [m_x m_x+3*s*cos(psi) m_x+3*s*cos(psi)-s*cos(psi-pi/4) m_x+3*s*cos(psi)-s*cos(psi+pi/4)];
%     y_w = [m_y m_y+3*s*sin(psi) m_y+3*s*sin(psi)-s*sin(psi-pi/4) m_y+3*s*sin(psi)-s*sin(psi+pi/4)];
%     line([x_w(1) x_w(2)],[y_w(1) y_w(2)],'color','b');
%     line([x_w(2) x_w(3)],[y_w(2) y_w(3)],'color','b');
%     line([x_w(2) x_w(4)],[y_w(2) y_w(4)],'color','b');
%     line([a(1) b(1)],[a(2) b(2)],'color','r');
%     sing = sign(sin(theta-psi));
%     if sing==0
%        sing = 1;
%     end
%     draw_boat([],s,m(1),m(2),theta,delta_r,sing*delta_sMax); 
%     line([x,x+0.05*axis_max_l*cos(alpha_cable)],[y,y+0.05*axis_max_l*sin(alpha_cable)],'color','c')
%     %delta_r
%     %delta_sMax
%     drawnow
% end

end