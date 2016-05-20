function [ vdot,omega_dot,W_ap1,W_ap2 ] = force_comp(v, a,delta_s,delta_r,omega,sizeSquare,theta,phi)

if length(theta)==1
    theta=theta*ones(sizeSquare);
end
if length(v)==1
    v=v*ones(sizeSquare);
end

if length(a)==1
    a=a*ones(sizeSquare);
end

if length(delta_s)==1
    delta_s=delta_s*ones(sizeSquare);
end
if length(delta_r)==1
    delta_r=delta_r*ones(sizeSquare);
end
if length(omega)==1
    omega=omega*ones(sizeSquare);
end

%Internal constants
p1 = 0.05;          % -         drift coefficientv_norm
p2 = .2;            %kg s^-1    tangential friction
p3 = 6000;          %kg m       angular friction
p4 = 1000;          %kg s^-1    sail lift
p5 = 2000;          %kg s^-1    rudder lift
p6 = 1;             %m          distance to sail CoE
p7 = 1;             %m          distance to mast
p8 = 2;             %m          distance to rudder
p9 = 300;           %kg         mass of boat
p10 = 10000;        %kg m^2     mass moment of intertia

if (nargin>7)
    if length(phi)==1
        phi=phi*ones(sizeSquare);
    end
    W_ap1 = (a.*cos(phi-theta)-v);
    W_ap2 = (a.*sin(phi-theta));
else
    W_ap1 = (a.*cos(theta)-v);
    W_ap2 = (a.*sin(theta));
end
%apperent wind speed vector in b-frame
phi_ap = atan2(W_ap2,W_ap1);    %apperent wind angle in b-frame
a_ap = sqrt(W_ap1.^2+W_ap2.^2);      %apperent wind speed velocity in b-frame


% sigma=cos(phi_ap)+cos(delta_s);
% for j=1:length(sigma(:,1))
%     for k = 1:length(sigma(1,:))
%         if (sigma(j,k)<0),
%             delta_s(j,k)=pi+phi_ap(j,k);
%         else
%             if sin(phi_ap(j,k))~=0
%                 delta_s(j,k)=-sign(sin(phi_ap(j,k))).*delta_s(j,k);
%             end
%         end
%     end
% end

Fs = p4*a_ap.*sin(delta_s-phi_ap);  %Force of wind on sail
Fr = p5*v.*sin(delta_r);             %Force of water on rudder

vdot = ((Fs.*sin(delta_s)-Fr.*sin(delta_r))-sign(v).*(p2*(v).^2))/p9;
omega_dot = (Fs.*(p6-p7*cos(delta_s))-p8*Fr.*cos(delta_r)...
    -p3*omega.*norm(v)...
     )/p10; 
end
