function [new_att,new_w,new_pos,new_vel] = forward_dynamics(att, w, pos, vel, Thrust, Mb, dt, J, m,g)
    % w : body axis angular rate vector
    p=w(1);
    q=w(2);
    r=w(3);
    phi = att(1); % roll
    theta = att(2); % pitch
    psi = att(3); % yaw
    
    w_hat = [0 -r  q;
             r  0 -p;
            -q  p  0];
    w_dot = inv(J)*(Mb - w_hat*J*w); % first derivative of w
    
    A = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi)           -sin(phi);
         0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
    
    att_dot = A*w; % first derivative of attitude
    
    phi_dot = att_dot(1);
    theta_dot = att_dot(2);
    
    A_dot = [0 (cos(phi)*tan(theta)*phi_dot)+(sin(phi)*sec(theta)*sec(theta)*theta_dot)  (cos(phi)*sec(theta)*sec(theta)*theta_dot)-(sin(phi)*tan(theta)*phi_dot);
             0 -sin(phi)*phi_dot                                                         -cos(phi)*phi_dot;
             0 (cos(phi)*sec(theta)*phi_dot)+(sin(phi)*sec(theta)*tan(theta)*theta_dot)  (cos(phi)*sec(theta)*tan(theta)*theta_dot)-(sin(phi)*sec(theta)*phi_dot)];
    
    att_double_dot = (A_dot*w) + (A*w_dot); % second derivative of w
    new_att_dot = att_dot + att_double_dot*dt; % new derivative of attitude
    new_att = att + new_att_dot*dt; % new value of the attitude
   
    Force = [(-1)*sin(theta)*m*g; cos(theta)*sin(phi)*m*g; -Thrust+(cos(theta)*cos(phi)*m*g)];
    
    vel_dot = (1/m)*Force - w_hat*vel;
    
    % Rot : Rotation matrix for transformation from body frame to inertial frame 
    Rot = [cos(theta)*cos(psi)  (sin(phi)*sin(theta)*cos(psi))-(cos(phi)*sin(psi))  (cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi));
         cos(theta)*sin(psi)  (sin(phi)*sin(theta)*sin(psi))+(cos(phi)*cos(psi))  (cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi));
        -sin(theta)            sin(phi)*cos(theta)                                 cos(phi)*cos(theta)];
    
    pos_dot = Rot*vel;
    
    R_dot = Rot*w_hat;
    
    pos_double_dot = R_dot*vel + Rot*vel_dot;
    new_pos_dot = pos_dot + pos_double_dot*dt;
    new_pos = pos + new_pos_dot*dt;
    
    phi = new_att(1);
    theta = new_att(2);
    psi = new_att(3);
    
    new_A = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
             0 cos(phi)           -sin(phi);
             0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
    
    new_Rot = [cos(theta)*cos(psi)  (sin(phi)*sin(theta)*cos(psi))-(cos(phi)*sin(psi))  (cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi));
               cos(theta)*sin(psi)  (sin(phi)*sin(theta)*sin(psi))+(cos(phi)*cos(psi))  (cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi));
              -sin(theta)            sin(phi)*cos(theta)                                 cos(phi)*cos(theta)];
    
    new_vel = inv(new_Rot)*new_pos_dot;
    
    new_w = inv(new_A)*new_att_dot;
end
