function [min_GTOW,val_S] = S_vs_GTOW()
    min_GTOW = 0;     % stores the minimum GTOW
    max_S = 6;
    x = 3:1:max_S;
    val_S = 3;        % Value of S at which GTOW will be minimum
    
    GTOW = zeros(1,max_S-2); % a vector to store the GTOW values
    i=3;
    while i<=max_S
        GTOW(i-2) = sizing_algo2(90,10,i,2);
        if i==3
            min_GTOW = GTOW(i-2);
        else 
            if min_GTOW > GTOW(i-2)
                min_GTOW = GTOW(i-2);
                val_S = i;
            end
        end
        i=i+1;
    end
    
    plot(x,GTOW);
    title('Plot of GTOW vs S keeping other parameters constant');
    xlabel('Number of battery cells');
    ylabel('GTOW');
    
end

