function plot_robot( robot,x )


[pT,pHip,p1R,p2R,p3R,p4R,p1L,p2L,p3L,p4L]=robot.get_joint_position(x);

plot([p3R(1),p1R(1),pHip(1),p2R(1),p4R(1)],[p3R(2),p1R(2),pHip(2),p2R(2),p4R(2)],'r','LineWidth',2)
hold on
plot([p3L(1),p1L(1),pHip(1),p2L(1),p4L(1)],[p3L(2),p1L(2),pHip(2),p2L(2),p4L(2)],'b','LineWidth',2)
plot([pT(1),pHip(1)],[pT(2),pHip(2)],'k','LineWidth',2);
hold off
end

