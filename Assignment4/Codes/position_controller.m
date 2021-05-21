function [Thrust, phi_des, theta_des] = position_controller(desired_pos, pos, att, vel,w, m, wn ,xi, g, max_phi, max_theta)
    p = w(1);
    q = w(2);
    r = w(3);
    phi = att(1);
    theta = att(2);
    psi = att(3);
    e = pos-desired_pos;
    
    w_cross = [0 -r  q; r  0 -p; -q  p  0];
    
    R = [cos(theta)*cos(psi)  (sin(phi)*sin(theta)*cos(psi))-(cos(phi)*sin(psi))  (cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi));
         cos(theta)*sin(psi)  (sin(phi)*sin(theta)*sin(psi))+(cos(phi)*cos(psi))  (cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi));
        -sin(theta)            sin(phi)*cos(theta)                                 cos(phi)*cos(theta)];
    
    e_dot = R*vel;
    R_dot = R*w_cross;
    e_ddot = -2*xi*wn*e_dot -  wn*wn*e; 
    vel_dot = inv(R)*(e_ddot - (R_dot*vel));
    
    F = m*(vel_dot + cross(w,vel));
    
    theta_des = -F(1)/(m*g);
    theta_des = min(max_theta, max(theta_des, -max_theta));
    
    phi_des = F(2)/(m*g);
    phi_des = min(max_phi, max(phi_des, -max_phi));
    
    Thrust = (cos(phi_des)*cos(theta_des)*m*g) - F(3);
    
end
