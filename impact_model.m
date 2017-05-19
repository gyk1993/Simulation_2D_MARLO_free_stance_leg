function x_a = impact_model(x)
global real_robot

x_a=zeros(14,1);
q=x(1:7);
dq=x(8:14);
q1=x(1); dq1=x(8);
q2=x(2); dq2=x(9);
q3=x(3); dq3=x(10);
q4=x(4); dq4=x(11);
q5=x(5); dq5=x(12);
q6=x(6); dq6=x(13);
q7=x(7); dq7=x(14);

qT=x(3);
q1R=x(4);
q2R=x(5);
q1L=x(6);
q2L=x(7);

L1=real_robot.L1; L2=real_robot.L2; L3=real_robot.L3; L4=real_robot.L4;

[EL,~]=real_robot.get_EL(x);

[D,C,G,B,~]= real_robot.Dynamic_model(q,dq);
[pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=real_robot.get_joint_position(x);

x_a(1:7)=x(1:7);

M=[D -EL'; EL zeros(2,2)];
temp=M^-1*[D*x(8:14);zeros(2,1)]; % the first seven element in temp is dq, last 2 is Fg]
x_a(8:14)=temp(1:7);

%switch leg
x_a([4,5,6,7])=x_a([6,7,4,5]);
x_a([11,12,13,14])=x_a([13,14,11,12]);

x_a([1 2])=p4L;
x_a([8 9])=[0;0];
end

