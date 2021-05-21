function [min_GTOW,val_AR] = AR_vs_GTOW()
    min_GTOW = 0;           % stores the minimum GTOW
    num_partition = 10;
    dx = 10/num_partition;
    x = 8:dx:17;
    val_AR = 8;             % Value of AR at which GTOW will be minimum
    
    GTOW = zeros(1,num_partition);  % a vector to store the GTOW values
     i=8;
     j=1;
    while i<=17
        GTOW(j) = sizing_algo2(90,i,4,2);
        if j==1
            min_GTOW = GTOW(j);
        else 
            if min_GTOW > GTOW(j)
                min_GTOW = GTOW(j);
                val_AR = i;
            end
        end
        j=j+1;
        i=i+1;
    end
    plot(x,GTOW);
    title('Plot of GTOW vs AR keeping other parameters constant');
    xlabel('Aspect ratio(AR)');
    ylabel('GTOW');

end

