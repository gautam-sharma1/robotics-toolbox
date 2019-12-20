function [EE_Pos]=project_workspace(Robot,qlim,N)
m=10000;
for i=1:m
    for j=1:N
        qq(j,i)=qlim(j,1)+(qlim(j,2)-qlim(j,1))*rand(1);
    end
    Mricx(i,:,:)=Robot.fkine(qq(:,i));
end
EE_Pos(:,:)=[Mricx(:,1,4),Mricx(:,2,4),Mricx(:,3,4)];
figure()
teach(Robot);
hold on;
plot3(Mricx(:,1,4),Mricx(:,2,4),Mricx(:,3,4),'b.','MarkerSize',5);
% drawnow;
% pause(0.001)
end