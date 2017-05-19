function [ LfLfhd LgLfhd ] = get_LfLfhd_LgLfhd( q,dq,alpha,theta_begin,theta_end,robot )
dhddq=get_dhddq(q,dq,alpha,theta_begin,theta_end);
d_dhddq=get_d_dhddq(q,dq,alpha,theta_begin,theta_end);

[D,C,G,B,~]= robot.Dynamic_model(q,dq);
[ER,dER]=robot.get_ER([q;dq]);
M=[D -ER'; ER zeros(2,2)];
H=[C*dq+G;dER*dq];
temp=dhddq*[eye(7) zeros(7,2)]*M^-1;

LfLfhd=d_dhddq*dq-temp*H;
LgLfhd=temp*[B;zeros(2,4)];

end