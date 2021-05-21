function [min_GTOW,val_DL] = DL_vs_GTOW()
    min_GTOW = 0;           % stores the minimum GTOW
    num_partition = 101;
    dx = 101/num_partition;
    x = 50:dx:150;
    val_DL = 50;            % Value of DL at which GTOW will be minimum
    
    GTOW = zeros(1,num_partition); % a vector to store the GTOW values
     i=50;
     j=1;
    while i<=150
        GTOW(j) = sizing_algo2(i,10,4,2);
        i=i+1;
        if j==1
            min_GTOW = GTOW(j);
        else 
            if min_GTOW > GTOW(j)
                min_GTOW = GTOW(j);
                val_DL = i;
            end
        end
        j=j+1;
    end
    plot(x,GTOW);
    title('Plot of GTOW vs DL keeping other parameters constant');
    xlabel('Disc Loading');
    ylabel('GTOW');
    min_GTOW
    val_DL
end

