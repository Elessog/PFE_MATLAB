function dy = boat_simulationS(t,y)

global rode_number max_windspeed ...
    time_const_wind psi delta_r delta_s delay command_buffer_size ...
    controller_freq buffer_command ...
    idx_bfc  delta_r_s delta_s_s;



%% Boat

 y_boat = [y(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+2)...
     ;y(4*3*rode_number+rode_number+4:4*3*rode_number+rode_number+5)...
     ;y(4*3*rode_number+rode_number+8)];

windspeed = max_windspeed*(1-exp(-t/time_const_wind));


if (t>=buffer_command(3,1)+delay)
    delta_r = buffer_command(1,1);
    delta_s = buffer_command(2,1);
    buffer_command(:,1:command_buffer_size-1) = buffer_command(:,2:command_buffer_size);
    idx_bfc = idx_bfc-1;
    if (idx_bfc==1)
        buffer_command(3,1) = t+1/controller_freq;
    end
end

dyt= model_sailboat_jaulinS(y_boat,windspeed,...
    psi,delta_s_s,delta_r_s);

dy = zeros(size(y));

dy(4*3*rode_number+rode_number+1:4*3*rode_number+rode_number+2) = dyt(1:2);
dy(4*3*rode_number+rode_number+4:4*3*rode_number+rode_number+5) = dyt(3:4);
dy(4*3*rode_number+rode_number+8) = dyt(5);
end

