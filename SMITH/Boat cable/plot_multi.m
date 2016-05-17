%plot

figure
hold on
axis square
axis([-10 100 -10 100])
plot(pos_boat(:,1),pos_boat(:,2),'g')
pointx = [0,20,40,60,30,0];
pointy = [0,0,20,10,0,0];
plot(pointx,pointy,'black')
plot(pos_boat_wt(:,1),pos_boat_wt(:,2),'r')
plot(y_s(66,:),y_s(67,:),'b')
plot(pos_to_controller(:,1),pos_to_controller(:,2),'c')
late = y_s(66:67,:)'+(pos_to_controller-pos_boat(:,1:2));
plot(late(:,1),late(:,2),'c');
legend('with smith','path','without smith','predictor','pose to control in smith')

figure
hold on
plot(x,sauv_delta(1,:),'g')
plot(x,sauv_delta_wt(1,:),'r')
title('rudder command with predictor or not')
legend('with smith','without smith')

figure
hold on
plot(x,sauv_delta(2,:),'g')
plot(x,sauv_delta_wt(2,:),'r')
title('rudder command with predictor or not')
legend('with smith','without smith')