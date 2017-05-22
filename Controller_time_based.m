classdef Controller_time_based < handle
    properties
        H0=[0, 0, 0, .5, .5,  0,  0;...
            0, 0, 0,  0,  0, .5, .5;...
            0, 0. 0, -1,  1,  0,  0;...
            0, 0, 0.  0.  0. -1,  1];
        c =[0, 0, -1, -.5, -.5, 0, 0];
        alpha = zeros(4,6);
        robot;
        
    end
    methods
        function u= input_torque_time_based( obj, x, t )
            global robot
            global alpha Tp
            global theta_begin theta_end
            global Kd Kp
            q=x(1:7);
            dq=x(8:14);
            
            h0=robot.H0*q;
            dh0=robot.H0*dq;
            
            s=t/Tp;
            hd = bezier(alpha,s);
            dhd = dbezier(alpha,s)*1/Tp;
            ddhd = d2bezier(alpha,s)*1/Tp^2;
            
            [D,C,G,B,~]= robot.Dynamic_model(q,dq);
            [ER,dER]=robot.get_ER(x);
            M=[D -ER'; ER zeros(2,2)];
            H=[C*dq+G;dER*dq];
            
            temp=robot.H0*[eye(7) zeros(7,2)]*M^-1;
            
            % u=(temp*[B;zeros(2,4)])^-1*(ddh+ddhd+temp*H);
            
            h=h0-hd;
            dh=dh0-dhd;
            ddh=-Kd*dh-Kp*h;
            
            Kp_u=500;
            Kd_u=100;
            u=-robot.H0(:,4:7)^-1*(Kp_u*h+Kd_u*dh);
            
            u(1)=10*(q(3)+0.2)+5*dq(3);
            
        end
    end
end