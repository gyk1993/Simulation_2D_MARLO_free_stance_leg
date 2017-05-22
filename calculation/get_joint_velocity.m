syms L1 L2 L3 L4 LT real

syms q1 q2 q3 q4 q5 q6 q7 real
syms dq1 dq2 dq3 dq4 dq5 dq6 dq7 real


yLeg=q1;
zLeg=q2;
qT=q3;
q1R=q4;
q2R=q5;
q1L=q6;
q2L=q7;

dyLeg=dq1;
dzLeg=dq2;
dqT=dq3;
dq1R=dq4;
dq2R=dq5;
dq1L=dq6;
dq2L=dq7;

th1R=qT+q1R;
th2R=qT+q2R;
th1L=qT+q1L;
th2L=qT+q2L;

p4R=[yLeg;zLeg];
p3R=p4R-R(th1R)*[0;(L4-L1)];
p2R=p4R-R(th1R)*[0;L4];
p1R=p3R-R(th2R)*[0;L3];

pHip=p1R-R(th1R)*[0;L1];

p1L=pHip+R(th1L)*[0;L1];
p2L=pHip+R(th2L)*[0;L2];
p3L=p1L+R(th2L)*[0;L3];
p4L=p2L+R(th1L)*[0;L4];

q=[q1;q2;q3;q4;q5;q6;q7];
dq=[dq1;dq2;dq3;dq4;dq5;dq6;dq7];

dpHip=jacobian(pHip,q)*dq;

function Rotation_matrix=R(x)
Rotation_matrix=[cos(x),-sin(x);sin(x),cos(x)];
end