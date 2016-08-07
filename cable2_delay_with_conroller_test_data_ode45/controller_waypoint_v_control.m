function [delta_r, delta_sMax] = controller_waypoint_v_control(x,y,theta,v_boat, psi,a, waypoints)
global previous_tack m_goingstarboard control_computed i_way
%controller_simpleLine Simple line following controller
%   Controller based on the paper "A simple controller for line 
%   following of sailboats" by Luc Jaulin and Fabrice Le Bars
%Constans/Parameters



  


m = [x,y];
%% value

m_xi_tack = pi/4;
max_tack = 60*pi/180;
minTackSpeed = 1;
m_starboardextreme = pi/4;
m_portextreme = -pi/4;
m_closereach = 5*pi/180;
m_running = pi/2;
m_maxCommandAngle = pi/2;
m_rudderspeedmin = 1;
m_sector_Angle = 5*pi/180;
%%
speed = v_boat;

%% set tack angle
xi_tack = m_xi_tack;

commandAngle = m_maxCommandAngle;

if speed < minTackSpeed
    xi_tack = (1-speed/minTackSpeed)*(max_tack-m_xi_tack)+m_xi_tack;
end

if speed>m_rudderspeedmin
   commandAngle = speed/m_rudderspeedmin*m_maxCommandAngle; 
end

%Step 3
waypoint = waypoints(i_way,:);
vect_to_waypoint  = waypoint(1:2)-m;

distance_to_waypoint = norm(vect_to_waypoint);
bearing_to_waypoint = atan2(vect_to_waypoint(2),vect_to_waypoint(1));
debR = wrapTo2Pi(psi+pi-xi_tack);
endR = wrapTo2Pi(psi+pi+xi_tack);
m_tack = isAngleinsector(bearing_to_waypoint,debR,endR);

if m_tack && previous_tack
    m_goingstarboard = ~isAngleinsector(bearing_to_waypoint,psi+pi-xi_tack,psi+pi);
end
if m_tack
    if ~continueDirection(m_goingstarboard,psi+pi,xi_tack,bearing_to_waypoint,distance_to_waypoint,m_sector_Angle,  waypoint)
        m_goingstarboard=~m_goingstarboard;
    end
        m_courseTo_steer = psi+pi+(m_goingstarboard*2-1)*xi_tack;
     previous_tack = 1;
else
        m_courseTo_steer = bearing_to_waypoint;
end
   

    %% rudder command
offCourse = wrapToPi(m_courseTo_steer-theta);
m_maxCommandAngle=commandAngle;

if all(abs(offCourse) <= m_maxCommandAngle)
    delta_r = m_portextreme+(-offCourse+m_maxCommandAngle)*(m_starboardextreme-m_portextreme) / (m_maxCommandAngle*2);
elseif all(offCourse < m_maxCommandAngle)     
    delta_r = m_starboardextreme;
else
    delta_r = m_portextreme; 
end
  %% sail command
  
  %link equations
W_ap = [a*cos(psi-theta)-speed a*sin(psi-theta)];
%apperent wind speed vector in b-frame
phi_ap = atan2(W_ap(2),W_ap(1));    %apparent wind angle in b-frame

relWind = wrapTo2Pi(phi_ap+pi);

if all(relWind < pi/2)
    delta_sMax = m_closereach;
else 
    delta_sMax = m_closereach + (relWind -pi/2)*(m_running-m_closereach)/(135*pi/180);
end
    
if all(distance_to_waypoint<waypoint(3))
   i_way = mod(i_way,length(waypoints(:,1))) +1;
end


% if control_computed
%     fprintf('brg: %.2f wpt: %.0f tack: %.0f cToStr: %.2f ofC: %.2f dltr: %.2f dlts: %.2f relWd: %.2f\n',bearing_to_waypoint,i_way,double(m_tack),m_courseTo_steer,offCourse,delta_r,delta_sMax,relWind);
% end

end



