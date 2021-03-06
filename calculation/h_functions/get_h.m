function h = get_h(in1,in2,in3,theta_begin,theta_end)
%GET_H
%    H = GET_H(IN1,IN2,IN3,THETA_BEGIN,THETA_END)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    16-Dec-2016 14:48:15

alpha1_1 = in3(1);
alpha1_2 = in3(5);
alpha1_3 = in3(9);
alpha1_4 = in3(13);
alpha1_5 = in3(17);
alpha1_6 = in3(21);
alpha1_7 = in3(25);
alpha1_8 = in3(29);
alpha1_9 = in3(33);
alpha2_1 = in3(2);
alpha2_2 = in3(6);
alpha2_3 = in3(10);
alpha2_4 = in3(14);
alpha2_5 = in3(18);
alpha2_6 = in3(22);
alpha2_7 = in3(26);
alpha2_8 = in3(30);
alpha2_9 = in3(34);
alpha3_1 = in3(3);
alpha3_2 = in3(7);
alpha3_3 = in3(11);
alpha3_4 = in3(15);
alpha3_5 = in3(19);
alpha3_6 = in3(23);
alpha3_7 = in3(27);
alpha3_8 = in3(31);
alpha3_9 = in3(35);
alpha4_1 = in3(4);
alpha4_2 = in3(8);
alpha4_3 = in3(12);
alpha4_4 = in3(16);
alpha4_5 = in3(20);
alpha4_6 = in3(24);
alpha4_7 = in3(28);
alpha4_8 = in3(32);
alpha4_9 = in3(36);
alpha1_10 = in3(37);
alpha1_11 = in3(41);
alpha2_10 = in3(38);
alpha2_11 = in3(42);
alpha3_10 = in3(39);
alpha3_11 = in3(43);
alpha4_10 = in3(40);
alpha4_11 = in3(44);
q3 = in1(3,:);
q4 = in1(4,:);
q5 = in1(5,:);
q6 = in1(6,:);
q7 = in1(7,:);
t2 = q4.*(1.0./2.0);
t3 = q5.*(1.0./2.0);
t8 = theta_end-theta_begin;
t9 = q3+t2+t3+theta_begin;
t13 = 1.0./t8;
t14 = t9.*t13;
t4 = t14+1.0;
t5 = t4.^2;
t6 = t5.^2;
t7 = t6.^2;
t10 = t9.^2;
t11 = t10.^2;
t12 = t11.^2;
t15 = 1.0./t8.^10;
t16 = 1.0./t8.^2;
t17 = 1.0./t8.^3;
t18 = 1.0./t8.^4;
t19 = 1.0./t8.^5;
t20 = 1.0./t8.^6;
t21 = 1.0./t8.^7;
t22 = 1.0./t8.^8;
t23 = 1.0./t8.^9;
h = [t2+t3-alpha1_1.*t5.*t7-alpha1_3.*t7.*t10.*t16.*4.5e1-alpha1_9.*t5.*t12.*t22.*4.5e1-alpha1_11.*t10.*t12.*t15+alpha1_2.*t4.*t7.*t9.*t13.*1.0e1-alpha1_5.*t5.*t6.*t11.*t18.*2.1e2-alpha1_7.*t6.*t10.*t11.*t20.*2.1e2+alpha1_10.*t4.*t9.*t12.*t23.*1.0e1+alpha1_6.*t4.*t6.*t9.*t11.*t19.*2.52e2+alpha1_4.*t4.*t5.*t6.*t9.*t10.*t17.*1.2e2+alpha1_8.*t4.*t5.*t9.*t10.*t11.*t21.*1.2e2;q6.*(1.0./2.0)+q7.*(1.0./2.0)-alpha2_1.*t5.*t7-alpha2_3.*t7.*t10.*t16.*4.5e1-alpha2_9.*t5.*t12.*t22.*4.5e1-alpha2_11.*t10.*t12.*t15+alpha2_2.*t4.*t7.*t9.*t13.*1.0e1-alpha2_5.*t5.*t6.*t11.*t18.*2.1e2-alpha2_7.*t6.*t10.*t11.*t20.*2.1e2+alpha2_10.*t4.*t9.*t12.*t23.*1.0e1+alpha2_6.*t4.*t6.*t9.*t11.*t19.*2.52e2+alpha2_4.*t4.*t5.*t6.*t9.*t10.*t17.*1.2e2+alpha2_8.*t4.*t5.*t9.*t10.*t11.*t21.*1.2e2;-q4+q5-alpha3_1.*t5.*t7-alpha3_3.*t7.*t10.*t16.*4.5e1-alpha3_9.*t5.*t12.*t22.*4.5e1-alpha3_11.*t10.*t12.*t15+alpha3_2.*t4.*t7.*t9.*t13.*1.0e1-alpha3_5.*t5.*t6.*t11.*t18.*2.1e2-alpha3_7.*t6.*t10.*t11.*t20.*2.1e2+alpha3_10.*t4.*t9.*t12.*t23.*1.0e1+alpha3_6.*t4.*t6.*t9.*t11.*t19.*2.52e2+alpha3_4.*t4.*t5.*t6.*t9.*t10.*t17.*1.2e2+alpha3_8.*t4.*t5.*t9.*t10.*t11.*t21.*1.2e2;-q6+q7-alpha4_1.*t5.*t7-alpha4_3.*t7.*t10.*t16.*4.5e1-alpha4_9.*t5.*t12.*t22.*4.5e1-alpha4_11.*t10.*t12.*t15+alpha4_2.*t4.*t7.*t9.*t13.*1.0e1-alpha4_5.*t5.*t6.*t11.*t18.*2.1e2-alpha4_7.*t6.*t10.*t11.*t20.*2.1e2+alpha4_10.*t4.*t9.*t12.*t23.*1.0e1+alpha4_6.*t4.*t6.*t9.*t11.*t19.*2.52e2+alpha4_4.*t4.*t5.*t6.*t9.*t10.*t17.*1.2e2+alpha4_8.*t4.*t5.*t9.*t10.*t11.*t21.*1.2e2];
