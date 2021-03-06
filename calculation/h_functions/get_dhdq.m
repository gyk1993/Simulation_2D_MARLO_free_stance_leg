function dhdq = get_dhdq(in1,in2,in3,theta_begin,theta_end)
%GET_DHDQ
%    DHDQ = GET_DHDQ(IN1,IN2,IN3,THETA_BEGIN,THETA_END)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    16-Dec-2016 14:48:14

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
t8 = q4.*(1.0./2.0);
t9 = q5.*(1.0./2.0);
t2 = q3+t8+t9+theta_begin;
t3 = t2.^2;
t4 = t3.^2;
t5 = t4.^2;
t6 = theta_end-theta_begin;
t7 = 1.0./t6.^10;
t10 = 1.0./t6;
t15 = t2.*t10;
t11 = t15+1.0;
t12 = t11.^2;
t13 = t12.^2;
t14 = t13.^2;
t16 = 1.0./t6.^3;
t17 = 1.0./t6.^4;
t18 = 1.0./t6.^5;
t19 = 1.0./t6.^6;
t20 = 1.0./t6.^7;
t21 = 1.0./t6.^8;
t22 = 1.0./t6.^2;
t23 = 1.0./t6.^9;
t24 = alpha1_10.*t2.*t5.*t7.*5.0;
t25 = alpha1_2.*t10.*t11.*t14.*5.0;
t26 = alpha1_4.*t3.*t11.*t12.*t13.*t16.*1.8e2;
t27 = alpha1_4.*t2.*t3.*t12.*t13.*t17.*4.2e2;
t28 = alpha1_6.*t4.*t11.*t13.*t18.*6.3e2;
t29 = alpha1_6.*t2.*t4.*t13.*t19.*6.3e2;
t30 = alpha1_8.*t3.*t4.*t11.*t12.*t20.*4.2e2;
t31 = alpha1_8.*t2.*t3.*t4.*t12.*t21.*1.8e2;
t32 = alpha1_2.*t2.*t14.*t22.*4.5e1;
t33 = alpha1_10.*t5.*t11.*t23.*4.5e1;
t34 = t24+t25+t26+t27+t28+t29+t30+t31+t32+t33-alpha1_1.*t10.*t11.*t14.*5.0-alpha1_3.*t2.*t14.*t22.*4.5e1-alpha1_9.*t5.*t11.*t23.*4.5e1-alpha1_11.*t2.*t5.*t7.*5.0-alpha1_7.*t2.*t4.*t13.*t19.*6.3e2-alpha1_5.*t4.*t11.*t13.*t18.*6.3e2-alpha1_9.*t2.*t3.*t4.*t12.*t21.*1.8e2-alpha1_5.*t2.*t3.*t12.*t13.*t17.*4.2e2-alpha1_7.*t3.*t4.*t11.*t12.*t20.*4.2e2-alpha1_3.*t3.*t11.*t12.*t13.*t16.*1.8e2+1.0./2.0;
t35 = q3.*2.0;
t36 = theta_begin.*2.0;
t37 = q4+q5+t35+t36;
t38 = alpha2_10.*t2.*t5.*t7.*5.0;
t39 = alpha2_2.*t10.*t11.*t14.*5.0;
t40 = alpha2_4.*t3.*t11.*t12.*t13.*t16.*1.8e2;
t41 = alpha2_4.*t2.*t3.*t12.*t13.*t17.*4.2e2;
t42 = alpha2_6.*t4.*t11.*t13.*t18.*6.3e2;
t43 = alpha2_6.*t2.*t4.*t13.*t19.*6.3e2;
t44 = alpha2_8.*t3.*t4.*t11.*t12.*t20.*4.2e2;
t45 = alpha2_8.*t2.*t3.*t4.*t12.*t21.*1.8e2;
t46 = alpha2_2.*t2.*t14.*t22.*4.5e1;
t47 = alpha2_10.*t5.*t11.*t23.*4.5e1;
t48 = t38+t39+t40+t41+t42+t43+t44+t45+t46+t47-alpha2_1.*t10.*t11.*t14.*5.0-alpha2_3.*t2.*t14.*t22.*4.5e1-alpha2_9.*t5.*t11.*t23.*4.5e1-alpha2_11.*t2.*t5.*t7.*5.0-alpha2_7.*t2.*t4.*t13.*t19.*6.3e2-alpha2_5.*t4.*t11.*t13.*t18.*6.3e2-alpha2_9.*t2.*t3.*t4.*t12.*t21.*1.8e2-alpha2_5.*t2.*t3.*t12.*t13.*t17.*4.2e2-alpha2_7.*t3.*t4.*t11.*t12.*t20.*4.2e2-alpha2_3.*t3.*t11.*t12.*t13.*t16.*1.8e2;
t49 = alpha3_10.*t2.*t5.*t7.*5.0;
t50 = alpha3_2.*t10.*t11.*t14.*5.0;
t51 = alpha3_4.*t3.*t11.*t12.*t13.*t16.*1.8e2;
t52 = alpha3_4.*t2.*t3.*t12.*t13.*t17.*4.2e2;
t53 = alpha3_6.*t4.*t11.*t13.*t18.*6.3e2;
t54 = alpha3_6.*t2.*t4.*t13.*t19.*6.3e2;
t55 = alpha3_8.*t3.*t4.*t11.*t12.*t20.*4.2e2;
t56 = alpha3_8.*t2.*t3.*t4.*t12.*t21.*1.8e2;
t57 = alpha3_2.*t2.*t14.*t22.*4.5e1;
t58 = alpha3_10.*t5.*t11.*t23.*4.5e1;
t59 = alpha4_10.*t2.*t5.*t7.*5.0;
t60 = alpha4_2.*t10.*t11.*t14.*5.0;
t61 = alpha4_4.*t3.*t11.*t12.*t13.*t16.*1.8e2;
t62 = alpha4_4.*t2.*t3.*t12.*t13.*t17.*4.2e2;
t63 = alpha4_6.*t4.*t11.*t13.*t18.*6.3e2;
t64 = alpha4_6.*t2.*t4.*t13.*t19.*6.3e2;
t65 = alpha4_8.*t3.*t4.*t11.*t12.*t20.*4.2e2;
t66 = alpha4_8.*t2.*t3.*t4.*t12.*t21.*1.8e2;
t67 = alpha4_2.*t2.*t14.*t22.*4.5e1;
t68 = alpha4_10.*t5.*t11.*t23.*4.5e1;
t69 = t59+t60+t61+t62+t63+t64+t65+t66+t67+t68-alpha4_1.*t10.*t11.*t14.*5.0-alpha4_3.*t2.*t14.*t22.*4.5e1-alpha4_9.*t5.*t11.*t23.*4.5e1-alpha4_11.*t2.*t5.*t7.*5.0-alpha4_7.*t2.*t4.*t13.*t19.*6.3e2-alpha4_5.*t4.*t11.*t13.*t18.*6.3e2-alpha4_9.*t2.*t3.*t4.*t12.*t21.*1.8e2-alpha4_5.*t2.*t3.*t12.*t13.*t17.*4.2e2-alpha4_7.*t3.*t4.*t11.*t12.*t20.*4.2e2-alpha4_3.*t3.*t11.*t12.*t13.*t16.*1.8e2;
dhdq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,alpha1_1.*t10.*t11.*t14.*-1.0e1+alpha1_2.*t10.*t11.*t14.*1.0e1+alpha1_2.*t2.*t14.*t22.*9.0e1-alpha1_9.*t5.*t11.*t23.*9.0e1-alpha1_3.*t14.*t22.*t37.*4.5e1+alpha1_10.*t2.*t5.*t7.*1.0e1-alpha1_11.*t2.*t5.*t7.*1.0e1+alpha1_10.*t5.*t11.*t23.*9.0e1+alpha1_6.*t2.*t4.*t13.*t19.*1.26e3-alpha1_7.*t2.*t4.*t13.*t19.*1.26e3-alpha1_5.*t4.*t11.*t13.*t18.*1.26e3+alpha1_6.*t4.*t11.*t13.*t18.*1.26e3+alpha1_8.*t2.*t3.*t4.*t12.*t21.*3.6e2+alpha1_4.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha1_9.*t2.*t3.*t4.*t12.*t21.*3.6e2-alpha1_5.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha1_7.*t3.*t4.*t11.*t12.*t20.*8.4e2-alpha1_3.*t3.*t11.*t12.*t13.*t16.*3.6e2+alpha1_8.*t3.*t4.*t11.*t12.*t20.*8.4e2+alpha1_4.*t3.*t11.*t12.*t13.*t16.*3.6e2,alpha2_1.*t10.*t11.*t14.*-1.0e1+alpha2_2.*t10.*t11.*t14.*1.0e1+alpha2_2.*t2.*t14.*t22.*9.0e1-alpha2_9.*t5.*t11.*t23.*9.0e1-alpha2_3.*t14.*t22.*t37.*4.5e1+alpha2_10.*t2.*t5.*t7.*1.0e1-alpha2_11.*t2.*t5.*t7.*1.0e1+alpha2_10.*t5.*t11.*t23.*9.0e1+alpha2_6.*t2.*t4.*t13.*t19.*1.26e3-alpha2_7.*t2.*t4.*t13.*t19.*1.26e3-alpha2_5.*t4.*t11.*t13.*t18.*1.26e3+alpha2_6.*t4.*t11.*t13.*t18.*1.26e3+alpha2_8.*t2.*t3.*t4.*t12.*t21.*3.6e2+alpha2_4.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha2_9.*t2.*t3.*t4.*t12.*t21.*3.6e2-alpha2_5.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha2_7.*t3.*t4.*t11.*t12.*t20.*8.4e2-alpha2_3.*t3.*t11.*t12.*t13.*t16.*3.6e2+alpha2_8.*t3.*t4.*t11.*t12.*t20.*8.4e2+alpha2_4.*t3.*t11.*t12.*t13.*t16.*3.6e2,alpha3_1.*t10.*t11.*t14.*-1.0e1+alpha3_2.*t10.*t11.*t14.*1.0e1+alpha3_2.*t2.*t14.*t22.*9.0e1-alpha3_9.*t5.*t11.*t23.*9.0e1-alpha3_3.*t14.*t22.*t37.*4.5e1+alpha3_10.*t2.*t5.*t7.*1.0e1-alpha3_11.*t2.*t5.*t7.*1.0e1+alpha3_10.*t5.*t11.*t23.*9.0e1+alpha3_6.*t2.*t4.*t13.*t19.*1.26e3-alpha3_7.*t2.*t4.*t13.*t19.*1.26e3-alpha3_5.*t4.*t11.*t13.*t18.*1.26e3+alpha3_6.*t4.*t11.*t13.*t18.*1.26e3+alpha3_8.*t2.*t3.*t4.*t12.*t21.*3.6e2+alpha3_4.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha3_9.*t2.*t3.*t4.*t12.*t21.*3.6e2-alpha3_5.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha3_7.*t3.*t4.*t11.*t12.*t20.*8.4e2-alpha3_3.*t3.*t11.*t12.*t13.*t16.*3.6e2+alpha3_8.*t3.*t4.*t11.*t12.*t20.*8.4e2+alpha3_4.*t3.*t11.*t12.*t13.*t16.*3.6e2,alpha4_1.*t10.*t11.*t14.*-1.0e1+alpha4_2.*t10.*t11.*t14.*1.0e1+alpha4_2.*t2.*t14.*t22.*9.0e1-alpha4_9.*t5.*t11.*t23.*9.0e1-alpha4_3.*t14.*t22.*t37.*4.5e1+alpha4_10.*t2.*t5.*t7.*1.0e1-alpha4_11.*t2.*t5.*t7.*1.0e1+alpha4_10.*t5.*t11.*t23.*9.0e1+alpha4_6.*t2.*t4.*t13.*t19.*1.26e3-alpha4_7.*t2.*t4.*t13.*t19.*1.26e3-alpha4_5.*t4.*t11.*t13.*t18.*1.26e3+alpha4_6.*t4.*t11.*t13.*t18.*1.26e3+alpha4_8.*t2.*t3.*t4.*t12.*t21.*3.6e2+alpha4_4.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha4_9.*t2.*t3.*t4.*t12.*t21.*3.6e2-alpha4_5.*t2.*t3.*t12.*t13.*t17.*8.4e2-alpha4_7.*t3.*t4.*t11.*t12.*t20.*8.4e2-alpha4_3.*t3.*t11.*t12.*t13.*t16.*3.6e2+alpha4_8.*t3.*t4.*t11.*t12.*t20.*8.4e2+alpha4_4.*t3.*t11.*t12.*t13.*t16.*3.6e2,t34,t48,t49+t50+t51+t52+t53+t54+t55+t56+t57+t58-alpha3_1.*t10.*t11.*t14.*5.0-alpha3_3.*t2.*t14.*t22.*4.5e1-alpha3_9.*t5.*t11.*t23.*4.5e1-alpha3_11.*t2.*t5.*t7.*5.0-alpha3_7.*t2.*t4.*t13.*t19.*6.3e2-alpha3_5.*t4.*t11.*t13.*t18.*6.3e2-alpha3_9.*t2.*t3.*t4.*t12.*t21.*1.8e2-alpha3_5.*t2.*t3.*t12.*t13.*t17.*4.2e2-alpha3_7.*t3.*t4.*t11.*t12.*t20.*4.2e2-alpha3_3.*t3.*t11.*t12.*t13.*t16.*1.8e2-1.0,t69,t34,t48,t49+t50+t51+t52+t53+t54+t55+t56+t57+t58-alpha3_1.*t10.*t11.*t14.*5.0-alpha3_3.*t2.*t14.*t22.*4.5e1-alpha3_9.*t5.*t11.*t23.*4.5e1-alpha3_11.*t2.*t5.*t7.*5.0-alpha3_7.*t2.*t4.*t13.*t19.*6.3e2-alpha3_5.*t4.*t11.*t13.*t18.*6.3e2-alpha3_9.*t2.*t3.*t4.*t12.*t21.*1.8e2-alpha3_5.*t2.*t3.*t12.*t13.*t17.*4.2e2-alpha3_7.*t3.*t4.*t11.*t12.*t20.*4.2e2-alpha3_3.*t3.*t11.*t12.*t13.*t16.*1.8e2+1.0,t69,0.0,1.0./2.0,0.0,-1.0,0.0,1.0./2.0,0.0,1.0],[4,7]);
