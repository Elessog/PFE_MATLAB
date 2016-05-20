figure(100)

subplot(1,2,1)
view(3) 
subplot(1,2,2)
view(3) 
set(figure(100), 'Position', [100, 100, 2000, 1000], 'toolbar', 'figure' )
h = uicontrol('style','slider','Min',-3.14,'Max',3.14,'Value',0.00,'units','pixel','position',[100 10 1800 30]);

addlistener(h,'ActionEvent',@(hObject, event) printmesh(hObject, event));