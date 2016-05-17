function dy = boat_simulation(t,y)

global index_out max_windspeed ...
    time_const_wind psi delta_r delta_s delay command_buffer_size ...
    controller_freq size_rect_cont control_computed buffer_command ...
    idx_bfc  delta_r_s delta_s_s;



%% Boat

windspeed = max_windspeed*(1-exp(-t/time_const_wind));

[a,b,index_out] = path_planning_v_control(y(1),y(2),index_out);

if mod(t,1/controller_freq)<(1/controller_freq)*size_rect_cont
   if (~control_computed)
     [delta_r_s, delta_s_s] = controller_simpleLine_v_control(y(1),...
         y(2), y(3), y(4),psi, a, b);
     buffer_command(:,idx_bfc) = [delta_r_s; delta_s_s;t];
     idx_bfc = idx_bfc+1;
     control_computed = 1;
   end
else
    control_computed = 0;
end

if (t>=buffer_command(3,1)+delay)
    delta_r = buffer_command(1,1);
    delta_s = buffer_command(2,1);
    buffer_command(:,1:command_buffer_size-1) = buffer_command(:,2:command_buffer_size);
    idx_bfc = idx_bfc-1;
    if (idx_bfc==1)
        buffer_command(3,1) = t+1/controller_freq;
    end
end

dy= model_sailboat_jaulin(y,windspeed,...
    psi,delta_s,delta_r);

end

