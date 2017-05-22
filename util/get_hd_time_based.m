function [hd,dhd,s] = get_hd_time_based( q,dq,alpha,Tp,impact_times_index,T )
    j=0;
    step_ini=0;
    for i=1:length(T)
        if T(i)>T(impact_times_index(j+1))
            j=j+1;
            step_ini=T(impact_times_index(j));
        end
        s(i)=(T(i)-step_ini)/Tp;
        if s(i) > 1
            s(i) = 1;
        elseif s(i)<0
            s(i) = 0;
        end
    end
    hd = bezier(alpha,s);
    dhd = bezier(alpha,s);
end
