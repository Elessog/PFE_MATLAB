function [ angles ] = correction_angle( angles_deb,size_buff )
%CORRECTION_ANGLE take compass data and linearize it
% and filter outliers
angles_temp = angles_deb; 
diff_ang = diff(angles_deb);


for i=1:length(diff_ang)
    if abs(diff_ang(i))>6
        angles_temp(i+1) = angles_temp(i+1)-sign(diff_ang(i))*2*pi;
        diff_ang = diff(angles_temp);
    elseif abs(diff_ang(i))>0.5
        angles_temp(i+1) = angles_temp(i);
        diff_ang = diff(angles_temp);
    end
end

angles = zeros(1,length(angles_deb));
for i=1:length(angles_temp)
    angles(i) = mean(angles_temp(max(1,i-size_buff):min(i+size_buff,length(angles_temp))));
end

end

