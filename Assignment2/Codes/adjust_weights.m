function [weight_error,new_GTOW] = adjust_weights(endur,R,b_chord,Nb,p_weight,pre_GTOW,S)
    
    sigma = (Nb*b_chord)/(pi*(R));     % solidity
    ef = 0.8;                          % given that motor efficiency is 80%
    rho = 1.225;                       % density of air
    Cl_a = 5.73;                       % Lift Curve slope
    Cd = 0.01;                         % Drag coefficient
    p_angle_0 = 22;                    % blade pitch angle in degrees
    
    % Calculating thrust and torque coefficient using BEMT
    [Ct, Cq] = BEMT(p_angle_0,R,b_chord,Nb,Cl_a,Cd,10000,rho);
    
    w = sqrt(pre_GTOW*9.8/(4*1000*(Ct*rho*pi*R^4))); % omega
    rpm = w*60/(2*pi);                               % rotation per minute
    P = (rho*pi*(R^2)*Cq*((w*R)^3))/ef;              % power
    P_max = (rho*pi*(R^2)*Cq*((sqrt(2)*w*R)^3))/ef;  % maximum power
    
    V=S*3.7;                         % Total voltage of the battery
    R = R*100;                       % rotor radius converted to cm from m
    K_v = rpm/V;                     % Kv of the motor
    I = P/V;                         % current
    I_max = P_max/V;                 % maximum current
    C = 4*I*(endur/60)*1000;         % battery capacity
    m_R = (0.0195)*(R^2.0859)*(sigma^(-0.2038))*(Nb^0.5344); % rotor mass 
    m_B = (0.0418)*(C^0.9327)*(S^1.0725);                    % battery mass
    
    l_BL = (4.8910)*(I_max^0.1751)*(P_max^0.2476); % motor casing length
    d_BL = (41.45)*(K_v^-0.1919)*(P_max^0.1935);   % BLDC motor diameter dBL
    m_BL = (0.0109)*(K_v^0.5122)*(P_max^-0.1902)*((log10(l_BL))^2.5582)*((log10(d_BL))^12.8502);
                                                   % mass of BLDC motor
    m_EBL = (0.8013)*(I_max^0.9727);               % mass of ESC for BLDC motor
    
    m_A = 1.3119*(R^1.2767)*(m_B^0.4587);          % airframe mass
    
    new_GTOW = m_A + 4*m_EBL + 4*m_BL + p_weight + 4*m_R + m_B;
    weight_error = abs(new_GTOW - pre_GTOW);
end
