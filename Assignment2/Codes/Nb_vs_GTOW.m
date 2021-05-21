function [min_GTOW,val_Nb] = Nb_vs_GTOW()
    min_GTOW = 0;       % stores the minimum GTOW
    max_Nb = 3;
    x = 2:1:max_Nb;
    val_Nb = 2;         % Value of Nb at which GTOW will be minimum
    
    GTOW = zeros(1,max_Nb-1); % a vector to store the GTOW values
    i=2;
    while i<=max_Nb
        GTOW(i-1) = sizing_algo2(90,10,4,i);
        if i==2
            min_GTOW = GTOW(i-1);
        else 
            if min_GTOW > GTOW(i-1)
                min_GTOW = GTOW(i-1);
                val_Nb = i;
            end
        end
        i=i+1;
    end
    
    plot(x,GTOW);
    title('Plot of GTOW vs Nb keeping other parameters constant');
    xlabel('Number of blades per rotor(Nb)');
    ylabel('GTOW');
    
end

