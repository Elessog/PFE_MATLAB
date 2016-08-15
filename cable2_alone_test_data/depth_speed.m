

%% depth depending on length
rho = 1000;
radius=0.005;
g=9.81;
CD=1.2;

depth_all = [];

L=3:3:20;
C = [1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1];
h = zeros(1, length(L));
s = cell(1, length(L));


v_test = 0:0.01:3;
figure(1)
depth_all = [];
for i=1:length(L)
    L_ = L(i);
    dl = 0.140;
    ms=dl*L_;
    depth_test =-cos(atan(CD*2*radius*L_*rho*v_test.^2/(2*g*(rho*pi*L_*radius^2-ms))))*L_;
    depth_all = [depth_all; depth_test];
    s{i} = sprintf('$%d~m$', L_);
end

h = legend(plot(v_test,depth_all), s);
title('Depth of cable over speed Pendulum')
hx = xlabel('Speed $m$/$s$');
hy = ylabel('Depth $m$');


v = get(h,'title');
set(v,'string','Length of Cable');
set(h,'Interpreter','latex')
set(hx,'Interpreter','latex')
set(hy,'Interpreter','latex')



%% depth depending on mass

lm =0.100:0.2:1;

C = [1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1];
h = zeros(1, length(lm));
s = cell(1, length(lm));

v_test = 0:0.01:3;
figure(2)
depth_all = [];
for i=1:length(lm)
    L_ = 10;
    dl = lm(i);
    ms=dl*L_;
    depth_test =-cos(atan(CD*2*radius*L_*rho*v_test.^2/(2*g*(rho*pi*L_*radius^2-ms))))*L_;
    depth_all = [depth_all; depth_test];
    s{i} = sprintf('$%.1f~kg \\slash m$', lm(i));
end

h = legend(plot(v_test,depth_all), s);
title('Depth of cable over speed 10 m Pendulum')
hx = xlabel('Speed $m$/$s$');
hy = ylabel('Depth $m$');

v = get(h,'title');
set(v,'string','Linear Mass of Cable');

set(h,'Interpreter','latex')
set(hx,'Interpreter','latex')
set(hy,'Interpreter','latex')


%%

figure(1)
print -depsc depth_length_speed_pendulum

figure(2)
print -depsc depth_mass_speed_pendulum

