function [Ct, Cq] = BEMT(p_angle_0,b_radius,b_chord,b_Nb,b_Cla,b_Cd,num_partition,rho)

    % Copying some variables in other smaller variables so that the 
    % equations used later do not look bulky.
    Cla = b_Cla;                 % Lift curve slope, Cl_α
    Cd = b_Cd;                   % Drag Coefficient
    Nb = b_Nb;                   % Number of blades per rotor
    c = b_chord;                 % Blade Chord
    R = b_radius;                % Blade radius

    sigma= (Nb*c)/(pi*(R));      % solidity
    % omega = rpm*(2*pi)/60;        % Angular velocity in rad/sec
    dr=1/num_partition;          % Number of partitions

    r=0:dr:1;
    Ct=0;                        % Thrust Coefficient
    Cq_induced=0;                % Torque Coefficient(for induced power)
    Cq_profile = 0;              % Torque Coefficient(for profile power)

    for i = 1:length(r)
        p_angle = (p_angle_0 - (17*r(i)))*pi/180;
        lambda=((sigma*Cla)/16)*(sqrt(1+(32*p_angle*r(i))/(sigma*Cla))-1);
        % lambda is inflow
        dCt=(0.5)*(sigma*Cla)*(p_angle*(r(i)^2) - lambda*r(i))*dr;
        % dCt is small change in Ct

        dCq_induced = lambda*dCt;
        % dCq_induced is small change in Cq_induced
        Ct=dCt + Ct;
        Cq_induced = Cq_induced + dCq_induced;
    end
    %thrust = rho*pi*(b_radius^2)*Ct*(omega*b_radius)^2;

    for i = 1:length(r)
        dCq_profile = (0.5*sigma*Cd*(r(i)^3)*dr);
        % dCq_profile is small change in Cq_profile
        Cq_profile = Cq_profile + dCq_profile;
    end
    Cq = Cq_profile + 1.15 * Cq_induced;
    %power = Cq * rho * pi*(b_radius^2)*(omega*b_radius)^3;
end
