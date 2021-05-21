function [l,m,n] = attitude_controller(att,att_d,w)
    zi = 0.7; % given
    w_n = 5; % given
    J = [0.01 0    0;
         0    0.05 0;
         0    0    0.001]; %given
    phi=att(1); 
    theta=att(2);
    psi=att(3);

    phi_d=att_d(1); % desired phi
    theta_d=att_d(2); % desired theta
    psi_d=att_d(3); % desired shi

    % w is body axis angular rate vector
    p=w(1);
    q=w(2);
    r=w(3);
  
    w_hat = [0 -r q;
             r  0 -p;
            -q p 0];
    A = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi)           -sin(phi);
         0 sin(phi)*sec(theta) cos(phi)*sec(theta)];

    e = att - att_d; % error
    e_dot = A*w; % first derivative of error

    phi_dot = e_dot(1); % first derivative of phi
    theta_dot = e_dot(2); % first derivative of theta
    psi_dot = e_dot(3); % first derivative of psi

    % A_dot : first derivative of A
    A_dot = [0 cos(phi)*tan(theta)*phi_dot + sin(phi)*sec(theta)*sec(theta)*theta_dot cos(phi)*sec(theta)*sec(theta)*theta_dot - sin(phi)*tan(theta)*phi_dot;
             0 -sin(phi)*phi_dot                                                      -cos(phi)*phi_dot;
             0 sin(phi)*sec(theta)*tan(theta)*theta_dot+cos(phi)*sec(theta)*phi_dot   cos(phi)*sec(theta)*tan(theta)*theta_dot - sin(phi)*sec(theta)*phi_dot];
    
    e_double_dot = (-2)*zi*w_n*e_dot - w_n*w_n*e; % second derivative of error
    w_dot = inv(A)*((e_double_dot) - A_dot*w); % first derivative of w

    tmp =(J*w_dot) + w_hat*J*w; % moments
    l = tmp(1); % rolling moment
    m = tmp(2); % pitching moment
    n = tmp(3); % yawing moment
end
