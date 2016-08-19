%plot
close all;
if ~exist('run_charged','var')
    run_charged = 0;
else
    choice =questdlg('Would you want to restart simulations?','Yes','No');
    switch choice
        case 'Yes'
            run_charged = 0;
    end
end

if ~run_charged
    clear all
    [FileName1,PathName] = uigetfile('*.mat','Select the real run');
    
    load([PathName,FileName1]);

    waypoints_1 = waypoints;
    time_1 = time;
	east_north_1 = east_north;
	heading_1 = heading;
	heading2_1= heading2;
	yaw_2_1 = yaw_2;
	yaw_1 = yaw;
	v_1 = v;
    windspeed_1 = windspeed;
    tw_d_1 = tw_d;
    arduino_1 =arduino;
    press1 = -3*(arduino_1(1,:)-181)/(378-181);
    
    [FileName3,PathName3] = uigetfile('*.mat','Select the simulation run');
    
    load([PathName3,FileName3]);


    waypoints_2 = waypoints;
    time_2_simu = time_2;
    time_2 = time;
	east_north_2 = east_north;
	heading_2 = heading;
	heading2_2= heading2;
	yaw_2_2 = yaw_2;
	yaw_2 = yaw;
    v_2_simu = v_2;
	v_2 = v;
    windspeed_2 = windspeed;
    tw_d_2 = tw_d;
    arduino_2 =arduino;
    press2 = -3*(arduino_2(1,:)-181)/(378-181);
    
    run_charged = 1;
end



%% Depth


rho = 1000;
radius=0.005;
ms=sum(m);
g=9.81;
CD=1.2;
L_=sum(L);


v_compute =v_real;% min(v_1):0.1:max(v_1);

sparsing = 1;
depth_comp = -cos(atan(CD*2*radius*L_*rho*v_compute.^2/(2*g*(rho*pi*(L_*radius^2+0.133*0.024^2)-ms))))*L_-0.3;

angle_ = atan(CD*2*radius*rho*L_*v_1(1:sparsing:end)/(2*(rho*pi*(L_*radius^2+0.133*0.024^2)-ms)));

depth1 =(press1+0.3)*sum(L)/3-0.3;

depth1_x= zeros(1,length(time_2_simu)-1);
depth_comp_2 =-cos(atan(CD*2*radius*L_*rho*v_2_simu.^2/(2*g*(rho*pi*(L_*radius^2+0.133*0.024^2)-ms))))*L_-0.3; 

for i=1:length(x)-1
   time_idx = find(time>=x(i),1,'first');
   time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
   depth1_x(i) =lagrange(x(i),time(time_vec),depth1(time_vec));
end

figure
hold on
plot(time_1-time_1(1),depth1,'g')
plot(time_2_simu-time_2_simu(1),rod_end_n,'r--')
plot(time_1-time_1(1),depth_comp,'b:','LineWidth',2)
t=title(['Depth cable over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
legend('Real Test','Simulation','Pendulum')

depth1_x_half= zeros(1,length(time_2_simu)-1);
depth_comp_2_half =-cos(atan(CD*2*radius*L_*rho*v_2_simu.^2/(2*g*(rho*pi*(L_*radius^2+0.133*0.024^2)-ms))))*L_/coeff_div_pressure_sensor-0.3; 

for i=1:length(x)-1
   time_idx = find(time>=x(i),1,'first');
   time_vec = max(1,time_idx-2):min(length(accel),time_idx+2);
   depth1_x_half(i) =lagrange(x(i),time(time_vec),press1(time_vec));
end
figure
hold on
sparsing = 2;
plot(time_1(1:sparsing:end)-time_1(1),press1(1:sparsing:end),'g')
plot(time_1(1:sparsing:end)-time_1(1),(depth_comp(1:sparsing:end)+0.3)/coeff_div_pressure_sensor-0.3,'b:','LineWidth',2)
sparsing = 10;
plot(time_2_simu(1:sparsing:end)-time_2_simu(1),rod_end_n_2(1:sparsing:end),'r--')
t=title(['Depth cable at the pressure sensor level over time ',FileName]);
set(t,'Interpreter','None')
xlabel('Time (s)')
ylabel('Depth (m)')
legend('Real Test','Pendulum','Simulation')


figure
hold on
v_compute = min(v_1):0.1:max(v_1);
sparsing = 1;
depth_comp = -cos(atan(CD*2*radius*L_*rho*v_compute.^2/(2*g*(rho*pi*(L_*radius^2+0.133*0.024^2)-ms))))*L_-0.3;


plot(v_1(1:sparsing:end),(press1(1:sparsing:end)+0.3)*sum(L)/3-0.3,'xg')
plot(v_2_simu(1:sparsing:end),rod_end_n(1:sparsing:end),'+r')
plot(v_compute,depth_comp,'b');
t=title(['Depth cable over speed ',FileName]);
set(t,'Interpreter','None')
xlabel('Speed (m/s)')
ylabel('Depth (m)')
legend('Real Test','Simulation','Pendulum')



figure
hold on 

sparsing = 5;
plot(v_1(1:sparsing:end),press1(1:sparsing:end),'xg')
sparsing = 20;
plot(v_2_simu(1:sparsing:end),rod_end_n_2(1:sparsing:end),'+r')
plot(v_2_simu(1:sparsing:end),depth_comp_2_half(1:sparsing:end),'ob');
t=title(['Depth cable at the pressure sensor level over speed ',FileName]);
set(t,'Interpreter','None')
xlabel('Speed (m/s)')
ylabel('Depth (m)')
legend('Real Test','Simulation','Pendulum')


