function res = continueDirection(m_goingstarboard,psi,xi_tack,bearing_to_waypoint,distance_to_waypoint, m_sector_Angle,waypoint)
if m_goingstarboard
    secBeg = psi-m_sector_Angle;
    secEnd = psi+xi_tack;
    extSecBeg = psi-xi_tack;
    extSecEnd = psi-m_sector_Angle;
else
    secBeg = psi-xi_tack;
    secEnd = psi+m_sector_Angle;
    extSecBeg = psi+m_sector_Angle;
    extSecEnd = psi+xi_tack;
end
res=0;
if isAngleinsector(bearing_to_waypoint,secBeg,secEnd)
    res = 1;
end
if isAngleinsector(bearing_to_waypoint,extSecBeg,extSecEnd)
    dist = waypoint(3)/sin(psi-m_sector_Angle);
    res = double(distance_to_waypoint<dist);
end
end

