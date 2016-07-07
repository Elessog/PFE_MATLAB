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
plot(y_s(1,:),y_s(2,:),'b')
plot(pos_to_controller(:,1),pos_to_controller(:,2),'c')
late = y_s(1:2,:)'+(pos_to_controller-pos_boat(:,1:2));
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


figure
subplot(1,2,1)
hold on
later =[zeros(2,200) y_s(1:2,1:length(y_s(1,:))-200)];
plot(x,later)
plot(x,y_s(1:2,:))
subplot(1,2,2)
plot(x,y_s(1:2,:)-later)