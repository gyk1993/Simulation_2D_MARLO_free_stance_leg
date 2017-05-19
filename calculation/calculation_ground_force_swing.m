clear all

q=sym('q',[7,1],'real');
dq=sym('dq',[7,1],'real');
u=sym('u',[4,1],'real');
ddq=sym('ddq',[7,1],'real');
F=sym('F',[2,1],'real');
syms L1 L2 L3 L4 real

qT=q(3);
q1R=q(4);
q2R=q(5);
q1L=q(6);
q2L=q(7);

ER=[ 1, 0
 0, 1
 0, 0
 0, 0
 0, 0
 0, 0
 0, 0];
 
 
 EL=[                                                                       1,                                                                       0
                                                                       0,                                                                       1
 L2*cos(q2R + qT) + L4*cos(q1R + qT) - L2*cos(q2L + qT) - L4*cos(q1L + qT), L2*sin(q2R + qT) + L4*sin(q1R + qT) - L2*sin(q2L + qT) - L4*sin(q1L + qT)
                                                         L4*cos(q1R + qT),                                                         L4*sin(q1R + qT)
                                                         L2*cos(q2R + qT),                                                         L2*sin(q2R + qT)
                                                       -L4*cos(q1L + qT),                                                       -L4*sin(q1L + qT)
                                                       -L2*cos(q2L + qT),                                                       -L2*sin(q2L + qT)];
EL=EL';
ER=ER';

v_stance=ER*dq;
v_swing=EL*dq;
dER=jacobian(v_stance,q);
dEL=jacobian(v_swing,q);

dER=[ 0, 0, 0, 0, 0, 0, 0
 0, 0, 0, 0, 0, 0, 0];
dEL=[ 0, 0, L2*dq7*sin(q3 + q7) - L2*dq5*sin(q3 + q5) - L4*dq4*sin(q3 + q4) - dq3*(L2*sin(q3 + q5) + L4*sin(q3 + q4) - L2*sin(q3 + q7) - L4*sin(q3 + q6)) + L4*dq6*sin(q3 + q6), - L4*dq3*sin(q3 + q4) - L4*dq4*sin(q3 + q4), - L2*dq3*sin(q3 + q5) - L2*dq5*sin(q3 + q5),   L4*dq3*sin(q3 + q6) + L4*dq6*sin(q3 + q6),   L2*dq3*sin(q3 + q7) + L2*dq7*sin(q3 + q7)
 0, 0, dq3*(L2*cos(q3 + q5) + L4*cos(q3 + q4) - L2*cos(q3 + q7) - L4*cos(q3 + q6)) + L2*dq5*cos(q3 + q5) + L4*dq4*cos(q3 + q4) - L2*dq7*cos(q3 + q7) - L4*dq6*cos(q3 + q6),   L4*dq3*cos(q3 + q4) + L4*dq4*cos(q3 + q4),   L2*dq3*cos(q3 + q5) + L2*dq5*cos(q3 + q5), - L4*dq3*cos(q3 + q6) - L4*dq6*cos(q3 + q6), - L2*dq3*cos(q3 + q7) - L2*dq7*cos(q3 + q7)];


