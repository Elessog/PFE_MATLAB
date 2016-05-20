function  printmesh(hObject, event)

phi = get(hObject,'Value');
subplot(1,2,1)
[az1,el1] = view;
subplot(1,2,2)
[az,el] = view;
a=5;
v =5;

theta = 0;

delta_s = -1.5:0.1:1.5;
delta_r = -1.5:0.1:1.5;
omega = 0;

[xx,yy] = meshgrid(delta_s,delta_r);
delta_s =xx;
delta_r = yy;
[vdot,omega_dot,w1,w2] = force_comp(v, a,delta_s,delta_r,omega,size(xx),theta-phi);

xlab = '\delta_s';
ylab = '\delta_r';
figure(100)

subplot(1,2,1)
surf(xx,yy,vdot)
colormap(jet)    % change color map
xlabel(xlab)
ylabel(ylab)
view(az1,el1) 
subplot(1,2,2)
surf(xx,yy,omega_dot)
colormap(jet)    % change color map
xlabel(xlab)
ylabel(ylab)
view(az,el) 
titl = sprintf('phi : %0.3f',phi);
title(titl)
hold off

end

