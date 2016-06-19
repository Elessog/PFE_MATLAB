function  printdeb(hObject, ~)
global i_end i_deb axis_mat time_g east_north_g v_g;

i_deb= floor(get(hObject,'Value'));

subplot(1,2,1)
[az1,el1] = view;
subplot(1,2,2)
[az2,el2] = view;

figure(100)

subplot(1,2,1)
plot(east_north_g(1,i_deb:i_end),east_north_g(2,i_deb:i_end));
axis square
axis(axis_mat);
xlabel('eastern');
ylabel('northen');
view(az1,el1) 


subplot(1,2,2)
plot(time_g(i_deb:i_end),v_g(i_deb:i_end))
xlabel('Time')
ylabel('Speed knot')
view(az2,el2) 

titl = sprintf('Time: %0.1f s - %0.1f s Duration: %0.1f ; index %d - %d',time_g(i_deb),time_g(i_end),time_g(i_end)-time_g(i_deb),i_deb,i_end);
title(titl)
hold off

end

