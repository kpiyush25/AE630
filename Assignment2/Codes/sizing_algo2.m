function final_GTOW = sizing_algo2(DL,asp_ratio,S,Nb)
    % fixed paramters:
    endur = 20;               % endurance or flight time
    N=4;                      % no. of rotors
    p_weight = 200;           % weight of the payload
    g = 9.8;                  % acceleration due to gravity
    
    % Initial guess:
    pre_GTOW = 3*p_weight;    % initial guess of the GTOW
    
    R = sqrt(pre_GTOW*g/(N*pi*DL*1000));  % rotor radius
    b_chord = R/asp_ratio;                % blade chord length
    
    % Iteration over the total weight begins and will end when the error
    % will become less than 5 grams
    [weight_error, new_GTOW] = adjust_weights(endur,R,b_chord, ...
        Nb,p_weight,pre_GTOW,S);
    
    while weight_error > 5
        pre_GTOW = new_GTOW;
        [weight_error, new_GTOW] = adjust_weights(endur,R,b_chord, ...
            Nb,p_weight,pre_GTOW,S);
    end

    final_GTOW = new_GTOW;
end

