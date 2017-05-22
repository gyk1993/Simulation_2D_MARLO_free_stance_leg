function [h0,dh0,s] = get_h0_time_based( q,dq,alpha,Tp,impact_times_index,T,H0 )
    h0 = H0*q;
    dh0 = H0*dq;
end