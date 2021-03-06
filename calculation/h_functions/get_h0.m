function h0 = get_h0(in1,in2,in3,theta_begin,theta_end)
%GET_H0
%    H0 = GET_H0(IN1,IN2,IN3,THETA_BEGIN,THETA_END)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    16-Dec-2016 14:49:19

q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
h0 = [q4.*(1.0./2.0)+q5.*(1.0./2.0);q6.*(1.0./2.0)+q7.*(1.0./2.0);-q4+q5;-q6+q7];
