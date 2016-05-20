function dy = boat_simulation_smith(t,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


global max_windspeed time_const_wind psi delta_r_s delta_s_s ;





%% Boat

windspeed = max_windspeed*(1-exp(-t/time_const_wind));

dy = model_sailboat_jaulin(y,windspeed,...
    psi,delta_s_s,delta_r_s);



end

