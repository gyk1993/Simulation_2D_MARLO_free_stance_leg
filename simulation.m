close all
clear all
addpath('./plot_functions');
addpath('./util');
addpath('./util/h_functions');
global robot
global Kd Kp
global Kd_u Kp_u
global alpha Tp
global theta_begin theta_end
global input_torque
global real_robot
global torso_angle d_torso_angle
global vel_des vel_avg vel_error_interate vel_error_derivative


input_torque=@input_torque_ml;

Kd=80*eye(4);
Kp=(Kd^2/5)*eye(4);

Kp_u=50;
Kd_u=5;

vel_des=0;

% First set
theta_begin=-3.28283938204512;
theta_end=-2.99861395834076;

load(['opt_result\avg_type1_' num2str(vel_des*10) 'dms'])

vel_des = outputs{1}.dq(end,1);

alpha=reshape(outputs{1}.a(1,:),4,6);
Tp=outputs{1}.t(1);
torso_angle=outputs{1}.q(:,3);
d_torso_angle=outputs{1}.dq(:,3);
% alpha=[3.2684	3.2878	3.2354	3.2111	3.1820	3.0640	3.1421	2.9644	3.0291	2.9366	2.9843
% 2.9844	3.0219	3.0384	3.1055	3.1102	3.1988	3.2000	3.2654	3.2757	3.2984	3.2689
% 0.8946	0.8498	0.8948	0.8349	0.9447	0.8034	0.9500	0.8247	0.8975	0.8522	0.8938
% 0.8944	0.9253	0.9923	1.1657	1.0272	1.5720	1.0313	1.6578	1.2176	1.3796	0.8946];
% 
% Tp=0.35;

% %% Second set
% theta_begin=-3.312576582646859;
% theta_end=-3.065299652762040;
% 
% IC=[0,0,0,3.5/4*pi,4.5/4*pi,pi,3/2*pi,0,0,0,0,0,0,0]';
% alpha=[3.449256411	3.431052439	3.341424859	3.308497057	3.152231298	3.202479478
% 3.202479954	3.23095169	3.363880499	3.517835299	3.619078486	3.449756355
% 0.874609764	0.86144129	0.900767619	0.868837293	0.885236728	0.923424111
% 0.924422801	0.946028098	1.358151483	1.267402083	1.318094727	0.874609731];

robot=Marlo_2D_class;
real_robot=Marlo_2D_class;
real_robot.mT=real_robot.mT;

IC=[0,0,-0.25/4*pi,3.5/4*pi,4.5/4*pi,5/4*pi,3/2*pi,   0,0,0,0,0,0,0]';
% IC=[0,0,0,3.5/4*pi,4.5/4*pi,3.5/4*pi,4.5/4*pi,   0,0,0,1/4*pi,1/4*pi,0,0]';
dt=0.01
timespan=0:dt:5;
opts = odeset('AbsTol',1e-12,'MaxStep',1e-2,'Events',@impactevent);
X=[];
T=[];
v_pstep=[];
count_step=[];
vel_avg=vel_des;
vel_error_interate=0;
vel_error_derivative=0;
vel_error_prev=0;
tstart=0;
index=[];
% [T,X]=ode45(@swing_model,timespan,IC);
for j=1:40
    j
    [TT,XX]=ode45(@swing_model,timespan,IC,opts);
    XX=XX';
    IC=impact_model(XX(:,end)); % leg switch is considered in the impact model
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(XX(:,1));
    pHip_1=pHip(1);
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(XX(:,end));
    pHip_2=pHip(1);
    dpHip=robot.get_joint_velocity(XX(:,end));
    vel_avg=dpHip(1);
    v_pstep=[v_pstep vel_avg*ones(1,length(TT))];
    vel_error_interate=vel_error_interate+(vel_avg-vel_des);
    vel_error_derivative=(vel_avg-vel_des)-vel_error_prev;
    vel_error_prev=(vel_avg-vel_des);
    %% for ml controller
%     alpha(1,:)=neuralnetwork_1(vel_avg)';
%     alpha(2,:)=neuralnetwork_2(vel_avg)';
%     alpha(3,:)=neuralnetwork_3(vel_avg)';
%     alpha(4,:)=neuralnetwork_4(vel_avg)';
    
    ctspeeddm=sign(vel_avg)*ceil(abs(vel_avg)*10);
    ctspeeddm=sign(ctspeeddm)*min(12,abs(ctspeeddm));
    if abs(ctspeeddm)>5
        tgspeeddm=sign(ctspeeddm)*(abs(ctspeeddm)-5);
    else
        tgspeeddm=0;
    end
    load(['opt_result\trans_type2_' num2str(ctspeeddm) 'to' num2str(tgspeeddm) 'dms']);
    torso_angle=outputs{1}.q(:,3);
    d_torso_angle=outputs{1}.dq(:,3);

    %%%%%%%%%%%%%%%
    X=[X XX];
    T=[T;tstart+TT];
    index(j)=length(T);
    count_step=[count_step j*ones(1,length(TT))];
    tstart=tstart+TT(end);
end



%% calculation for plot

% for i=1:length(T)
%     h0(:,i)=get_h0(X(1:7,i),X(8:14,i),alpha,theta_begin,theta_end);
% 
% end
[hd,dhd,s] = get_hd_time_based(X(1:7,:),X(8:14,:),alpha,Tp,index,T);
[h0,dh0] = get_h0_time_based(X(1:7,:),X(8:14,:),alpha,Tp,index,T,robot.H0);

h=h0-hd;
dh=dh0-dhd;

% u=-robot.H0(:,4:7)^-1*(Kp_u*h+Kd_u*dh);
% u=-(Kp_u*h+Kd_u*dh);

c =[0, 0, -1, -.5, -.5, 0, 0];
% for i=1:length(T)
% theta(i)=c*X(1:7,i);
% s(i)=(theta(i)-theta_begin)/(theta_end-theta_begin);
% end



for i=1:length(T)
    u(:,i)=input_torque(X(:,i),T(i));
end

% Last_five_step_time=T(index(end))-T(index(end-5))


%% begin plot
fig=figure(1);
clf(fig); 
color=['r','g','b','m'];
for j=1:4
    subplot(2,2,j)
    plot(T,hd(j,:),[color(j)]);
    hold on
    plot(T,h0(j,:),[color(j),'--']);
    hold off
    title('hd and h0');
end
% set(gcf,'NextPlot','add');
% axes;
% h = title('hd and h0');
% set(gca,'Visible','off');
% set(h,'Visible','on');

figure(2)
for j=1:4
    subplot(2,2,j)
    plot(T,h0(j,:)-hd(j,:));
%     axis([T(1),T(end),-0.1,0.1])
    title('y')
end

% set(gcf,'NextPlot','add');
% axes;
% h = title('h=hd-h0');
% set(gca,'Visible','off');
% set(h,'Visible','on');

figure(3)
for j=1:4
    subplot(2,2,j)
    plot(T,u(j,:))
%     axis([0,15,-20,20])
end


set(gcf,'NextPlot','add');
axes;
h = title('input u');
set(gca,'Visible','off');
set(h,'Visible','on');


figure(4)
plot(T,s)

mkdir(['image'])
h = get(0,'children');
for i=1:length(h)
    saveas(h(length(h)-i+1), ['./image/figure' num2str(i), '.bmp']);
end
title('s')


figure(5)
plot(T,v_pstep)
title('average velocity per step')


figure
for i=1:length(T)
    plot_robot(robot,X(:,i))
    hold on
    walk_range=-5:0.1:30;
    plot(walk_range,zeros(1,length(walk_range)),'LineWidth',2)
    hold off
    axis equal
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(X(:,i));
    axis([-2+pHip(1) +2+pHip(1) -1 3])
    text(pHip(1),2.5,{['current step:  ' num2str(count_step(i))],['left foot height:  '  num2str(p4L(2))],['Time: ' num2str(T(i))]});
    drawnow;
%     %%%%%%get video%%%%%
%     F(i) = getframe(gcf);
%     %%%%%%%%%
%     pause(dt);
end
%%%%%make video
% vi = VideoWriter('video', 'MPEG-4');
% set(vi, 'FrameRate', 1/dt);
% open(vi);
% writeVideo(vi, F);
% close(vi);
% %%%%%%%%%%%%%