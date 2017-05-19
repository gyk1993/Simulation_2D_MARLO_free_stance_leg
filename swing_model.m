function dx = swing_model( t,x )
global real_robot
global input_torque

dx=zeros(14,1);

q=x(1:7);
dq=x(8:14);

[D,C,G,B,~]= real_robot.Dynamic_model(q,dq);

% u=zeros(4,1);
u=input_torque(x);
Fg=ground_force_swing(real_robot,x,u);
[ER,dER]=real_robot.get_ER(x);

dx(1:7)=x(8:14);
dx(8:14)=D^-1*(-C*x(8:14)-G+B*u+ER'*Fg);
end



