function final_GTOW = sizing_algo()
    % fixed paramters:
    endur = 20;                % endurance or flight time
    N=4;                       % no. of rotors
    p_weight = 200;            % weight of the payload
    g = 9.8;                   % acceleration due to gravity
    
    % Initial guesses:
    Nb = 2;             % In my most optimal solution it remained 2
    DL = 90;            % In my most optimal solution it became 76
    S=4;                % In my most optimal solution it became 3
    asp_ratio = 10;     % In my most optimal solution it became 11
    pre_GTOW = 3*p_weight;
    
    R = sqrt(pre_GTOW*g/(N*pi*DL*1000)); % rotor radius
    b_chord = R/asp_ratio;               % blade chord length
    
    % Iteration over the total weight begins and will end when the error
    % will become less than 5 grams
    [weight_error, new_GTOW] = adjust_weights(endur,R,b_chord, ...
            Nb,p_weight,pre_GTOW,S);

    while weight_error > 5
        pre_GTOW = new_GTOW;
        [weight_error, new_GTOW] = adjust_weights(endur,R,b_chord, ...
            Nb,p_weight,pre_GTOW,S);
    end

    final_GTOW = new_GTOW

end

