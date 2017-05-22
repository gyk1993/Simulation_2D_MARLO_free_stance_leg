function u= input_torque_time_based( x,t )
global robot
global alpha Tp
global theta_begin theta_end
global Kd Kp
global Kd_u Kp_u
global torso_angle d_torso_angle
global vel_des vel_avg vel_error_interate vel_error_derivative
q=x(1:7);
dq=x(8:14);

h0=robot.H0*q;
dh0=robot.H0*dq;

dpHip=robot.get_joint_velocity(x);
alpha(1,:)=neuralnetwork_1(dpHip(1))';
alpha(2,:)=neuralnetwork_2(dpHip(1))';
alpha(3,:)=neuralnetwork_3(dpHip(1))';
alpha(4,:)=neuralnetwork_4(dpHip(1))';


s=t/Tp;
hd = bezier(alpha,s);
dhd = dbezier(alpha,s)*1/Tp;
ddhd = d2bezier(alpha,s)*1/Tp^2;
if s > 1
    s = 1;
    hd = bezier(alpha,s);
    dhd=0;
elseif s<0
    s = 0;
    hd = bezier(alpha,s);
    dhd=0;
end

% hd = bezier(alpha,s);
% dhd = dbezier(alpha,s)*1/Tp;
% ddhd = d2bezier(alpha,s)*1/Tp^2;

h=h0-hd;
dh=dh0-dhd;
ddh=-Kd*dh-Kp*h;

[D,C,G,B,~]= robot.Dynamic_model(q,dq);
[ER,dER]=robot.get_ER(x);
M=[D -ER'; ER zeros(2,2)];
H=[C*dq+G;dER*dq];

temp=robot.H0*[eye(7) zeros(7,2)]*M^-1;

% u=(temp*[B;zeros(2,4)])^-1*(ddh+ddhd+temp*H);



q3_desire=interp1(linspace(0,1,29),torso_angle,s);
dq3_desire=interp1(linspace(0,1,29),d_torso_angle,s);


dq3_desire=0;

dpHip=robot.get_joint_velocity(x);

h(1) = -(q(3)-q3_desire); % replace stance leg to torso angle
dh(1) = -(dq(3)-dq3_desire);
% h(3)=h(3)+0.1;
% h(2) = h(2)+0.25*(vel_des-dpHip(1));
% h(2) = h(2)-0.2*(vel_avg-vel_des)-0.01*vel_error_interate+0.05*vel_error_derivative;
% h(2) = h(2)-0.2*(vel_avg-vel_des)+0.05*vel_error_derivative;
% h(2) = h(2)-0.05*(vel_avg-vel_des);
u=-robot.H0(:,4:7)^-1*(Kp_u*h+Kd_u*dh);

end

