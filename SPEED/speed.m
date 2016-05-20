close all;
a=5;
v =4;

theta = 0;
kj = 0;
phi_v = -3.14:0.01:3.14;
vect_sauv = zeros(8,length(phi_v));
do_plot = 0;

for phi=phi_v;
    delta_s = -1.5:0.01:1.5;
    delta_r = -1.5:0.01:1.5;
    omega = 0;

    [xx,yy] = meshgrid(delta_s,delta_r);
    delta_s =xx;
    delta_r = yy;
    [vdot,omega_dot,w1,w2] = force_comp(v, a,delta_s,delta_r,omega,size(xx),theta-phi);

    xlab = '\delta_s';
    ylab = '\delta_r';
    kj=kj+1;
    if do_plot
        figure(100+kj)

        clf
        subplot(1,2,1)
        surf(xx,yy,vdot)
        colormap(jet)    % change color map
        xlabel(xlab)
        ylabel(ylab)
        subplot(1,2,2)
        surf(xx,yy,omega_dot)
        colormap(jet)    % change color map
        xlabel(xlab)
        ylabel(ylab)
        hold off
    end
    [val1,idx1] = max(vdot);
    [val1,idx2] = max(val1(:));
    idx1 = idx1(idx2);
    idxo2 = idx2;
    [valo1,idxo1] = min(abs(omega_dot(100:220,idx2)));
    idxo1 = idxo1+99;
    %idxo1 = idxo1(idxo2);
    vect_sauv(:,kj) = [val1;delta_s(1,idx2);delta_r(idx1,1);valo1;delta_s(1,idxo2);delta_r(idxo1,1);vdot(idxo1,idxo2);omega_dot(idx1,idx2)];
end

figure(50)
plot(phi_v,vect_sauv)
h=legend('value max $\dot{v}$','$\delta_s \dot{v}$','$\delta_r \dot{v}$','value max $\dot{\omega}$','$\delta_s \dot{\omega}$',...
    '$\delta_r \dot{\omega}$','$\dot{v}$ at min $\dot{\omega}$','$\dot{\omega}$ at max $\dot{v}$');

 set(h,'Interpreter','latex')