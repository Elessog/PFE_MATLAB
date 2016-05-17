close all;clc;
global Wn1c Pn1c Wn1ca Wn1cb rode_number Nn1c Kpl Kdl Kil lambdainverse...
    vect_x vect_y vect_z boat_dot boat_dotdot;
global L Lg mg m;

rode_number = 5;%number of rods to simulate the cable

mass_vect = 0:0.5:10;
angle_cable = zeros(length(mass_vect),1);
force_end_cable = zeros(length(mass_vect),3);
index = 0;
vect_speed = 0:0.5:15;

speed = 1;
for mass =  mass_vect
%% initialization of the state of the boat
index = index+1
origin = [0;0;0];
boat_dot=[speed;0;0];
boat_dotdot=[0;0;0];
theta_0 = 0;
%% Matrix creation for the cable simulation
%b initial vector of the rods
% b_0 = [0 0 0
%     0  0 0
%     -1 -1 -1];
L=4*ones(rode_number,1);
L(rode_number) = 0.1;
Lg = 4*ones(rode_number*3,1);
Lg(rode_number*3-2:rode_number*3) = 4;
b_0 = zeros(3,rode_number);
b_0(1,:)=-L'.*cos(theta_0).*ones(1,rode_number);
b_0(2,:)=-L'.*sin(theta_0).*ones(1,rode_number);
dl = 0.5;
m = dl*L;
mg = dl*Lg;
m(length(m)) =m(length(m))+mass;
mg(length(mg)-2:length(mg))=m(length(m)) ;
r_0 = zeros(3,rode_number);
r_0(:,1) = b_0(:,1)/2.0;

for i=2:length(b_0(1,:))
    r_0(:,i) = origin+sum(b_0(:,1:i-1),2)+b_0(:,i)/2.0;
end

%vect_dir creation for easy force adding

vect_x = zeros(3*rode_number,1);
vect_y = zeros(3*rode_number,1);
vect_z = zeros(3*rode_number,1);
vect_x(1:3:length(vect_x))=1;
vect_y(2:3:length(vect_y))=1;
vect_z(3:3:length(vect_z))=1;
%creation of q1 and q2
q1 = reshape(r_0,3*rode_number,1);
q2 = reshape(b_0,3*rode_number,1);

%creation of Wn1c

I_blk_n = eye(3);
for i=1:rode_number-1
    I_blk_n=blkdiag(I_blk_n,eye(3));
end

I_blk_n_minus1 = eye(3);
for i=1:rode_number-2
    I_blk_n_minus1=blkdiag(I_blk_n_minus1,eye(3));
end
I_blk_nm1=I_blk_n_minus1;
I_blk_n_minus1 = horzcat(I_blk_n_minus1,zeros(length(I_blk_n_minus1(:,1)),3));
I_blk_n_minus1 = vertcat(zeros(3,length(I_blk_n_minus1(1,:))),I_blk_n_minus1);

Wn1ca=-I_blk_n;
Wn1cb=I_blk_n_minus1;

Wn1c = Wn1ca+Wn1cb;
%mass matrix

%construction of Pn1c
Pn1c=(1/2.0)*(I_blk_n_minus1+I_blk_n);

Nn1c=zeros(3*rode_number,1);
Nn1c(1:3)=origin;

%%% constraint controler coefficients %%
%%% for the cable simulation      %%%%%%
wnl=500;
zeta=1;
Kpl=wnl^2;
Kdl=2*zeta*wnl;
Kil=10000;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lambdainverse = ((Wn1c*Wn1c'-6*Pn1c*(Wn1ca'-Wn1cb'))^-1);%unchanging part of the lagrangian multipliers calculation


%% creation vector y for the differential solving
y0 = vertcat(q2,q1);
y0 = vertcat(y0,zeros(size(q2)));
y0 = vertcat(y0,zeros(size(q1)));
y0 = vertcat(y0,zeros(rode_number,1));
y0 = vertcat(y0,[origin;0;0;0]);

%%%%%% Time parameters %%%%%%%
stepH = 0.001;
x= 0:stepH:30;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = zeros(length(y0),length(x));
y(:,1) = y0;                                          % initial condition
F_xy =@(t,y) boat_cable_simulation4(t,y);                    % change the function as you desire

f_cable= zeros(3,length(x));
%% computation of the ODE
for i=1:length(x)-1                            % calculation loop
    k_1 = F_xy(x(i),y(:,i));
    k_2 = F_xy(x(i)+0.5*stepH,y(:,i)+0.5*stepH*k_1);
    k_3 = F_xy((x(i)+0.5*stepH),(y(:,i)+0.5*stepH*k_2));
    [k_4,force_cable] = F_xy((x(i)+stepH),(y(:,i)+k_3*stepH));
    f_cable(:,i+1)=force_cable;
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*stepH;  % main equation
end
f_cable=f_cable';
force_end_cable(index,:) = f_cable(30001,:);
y= y';


%% data reorganization
b = y(:,1:3*rode_number);
r = y(:,3*rode_number+1:2*3*rode_number);
bdot = y(:,2*3*rode_number+1:3*3*rode_number);
rdot = y(:,3*3*rode_number+1:4*3*rode_number);
errorint = y(:,4*3*rode_number+1:4*3*rode_number+rode_number);
pos_boat = y(:,4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+3);

errorLdot = zeros(length(x),rode_number);% compute the error of the simulation on the length of the cable
for j=1:length(x)
    for k=1:rode_number
        errorLdot(j,k) = norm(b(j,1+(k-1)*3:k*3))-L(rode_number);
    end
end

angle_cable(index) =- (atan2(b(30001,3),b(30001,1))+pi/2)*180/pi;


end

%% visu
% 
% %v = VideoWriter('newfile.avi','Uncompressed AVI');
% %aviobj = avifile('example_osci.avi','compression','None','fps',25);
% 
% figure(666)
% jk = 0;
% ratio = 40;
% for i=1:ratio:length(x)-1
%     %% cable
%     jk = jk+1;
%     clf
%     
%     pos_b = pos_boat(i,:);
%     l = sum(L);
%     axis([-l+pos_b(1) l+pos_b(1)...
%         -l+pos_b(2) l+pos_b(2)...
%         -l+pos_b(3) 2+pos_b(3)])
%     axis vis3d
%     l = pos_boat(i,:);
%     for number_body=1:rode_number
%         
%         point=pos_boat(i,:);
%         for j=1:number_body
%             point = point+b(i,1+(j-1)*3:j*3);
%         end
%         l = [l; point];
%     end
%     
%     draw_cable(l,666,['r','g','b'])
%     title(sprintf('Time : %.3f',i*stepH));
%     drawnow
%     %aviobj = addframe(aviobj,gcf);     
%     pause(ratio*stepH)
% end
% 
% 
% viobj = close(aviobj)
