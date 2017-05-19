close all
clear all
addpath('./plot_functions');
addpath('./util');
addpath('./util/h_functions');
global robot
global Kd Kp
global alpha
global theta_begin theta_end
global input_torque
global real_robot

input_torque=@input_torque_IOL_1;

Kd=80*eye(4);
Kp=(Kd^2/5)*eye(4);


% First set
theta_begin=-3.28283938204512;
theta_end=-2.99861395834076;
IC=[0,0,0,3.5/4*pi,4.5/4*pi,pi,3/2*pi,0,0,0,0,0,0,0]';
alpha=[3.2684	3.2878	3.2354	3.2111	3.1820	3.0640	3.1421	2.9644	3.0291	2.9366	2.9843
2.9844	3.0219	3.0384	3.1055	3.1102	3.1988	3.2000	3.2654	3.2757	3.2984	3.2689
0.8946	0.8498	0.8948	0.8349	0.9447	0.8034	0.9500	0.8247	0.8975	0.8522	0.8938
0.8944	0.9253	0.9923	1.1657	1.0272	1.5720	1.0313	1.6578	1.2176	1.3796	0.8946];

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

IC=[0,0,0,3.5/4*pi,4.5/4*pi,pi,3/2*pi,0,0,0,0,0,0,0]';
dt=0.05
timespan=0:dt:10;
opts = odeset('AbsTol',1e-12,'MaxStep',1e-2,'Events',@impactevent);
X=[];
T=[];
tstart=0;
index=[];
% [T,X]=ode45(@swing_model,timespan,IC);
for j=1:30
    j
    [TT,XX]=ode45(@swing_model,timespan,IC,opts);
    XX=XX';
    IC=impact_model(XX(:,end)); % leg switch is considered in the impact model
    X=[X XX];
    T=[T;tstart+TT];
    index(j)=length(T);
    tstart=tstart+TT(end);
end



%% calculation for plot
for i=1:length(T)
    h0(:,i)=get_h0(X(1:7,i),X(8:14,i),alpha,theta_begin,theta_end);
    hd(:,i)=get_hd(X(1:7,i),X(8:14,i),alpha,theta_begin,theta_end);
end
c =[0, 0, -1, -.5, -.5, 0, 0];
for i=1:length(T)
theta(i)=c*X(1:7,i);
s(i)=(theta(i)-theta_begin)/(theta_end-theta_begin);
end



for i=1:length(T)
    u(:,i)=input_torque(X(:,i));
end

Last_five_step_time=T(index(end))-T(index(end-5))


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
end
set(gcf,'NextPlot','add');
axes;
h = title('hd and h0');
set(gca,'Visible','off');
set(h,'Visible','on');

figure(2)
for j=1:4
    subplot(2,2,j)
    plot(T,h0(j,:)-hd(j,:));
    axis([T(1),T(end),-0.1,0.1])
end

set(gcf,'NextPlot','add');
axes;
h = title('h=hd-h0');
set(gca,'Visible','off');
set(h,'Visible','on');

figure(3)
for j=1:4
    subplot(2,2,j)
    plot(T,u(j,:))
    axis([0,15,-20,20])
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
for i=1:length(T)
    plot_robot(robot,X(:,i))
    hold on
    walk_range=-5:0.1:30;
    plot(walk_range,zeros(1,length(walk_range)),'LineWidth',2)
    hold off
    axis equal
    [pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(X(:,i));
    axis([-2+pHip(1) +2+pHip(1) -1 3])
    drawnow;
%     %%%%%%get video%%%%%
%     F(i) = getframe(gcf);
%     %%%%%%%%%
%     pause(dt);
end
%%%%%make video
vi = VideoWriter('video', 'MPEG-4');
set(vi, 'FrameRate', 1/dt);
open(vi);
writeVideo(vi, F);
close(vi);
%%%%%%%%%%%%%%