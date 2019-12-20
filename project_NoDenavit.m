function [DH,Robot] = project_NoDenavit(z_j,h,v,RP,N,qlim)

% %Theta in Radians
% prompt="Please enter the number of joints ";
% n=input(prompt);
% disp("Joint axis of a Revolute joint is the axis of rotation");
% disp("Joint axis of a a Prismatic joint is the axis of displacement");
% for i=1:n
%          
%          prompt ="Please enter the directions of Z axis of all joints ";
%          z_j(i,:)=input(prompt);
% end
n=N;
 z_j(n+1,:)=z_j(n,:);
 for i=1:n
    X(i,:)=cross(z_j(i,:),z_j(i+1,:));
 end
% 
% 
 for i=1:n-1
% prompt ="What is the "+i+"st joint [P/R] ";
% k=input(prompt,'s');


if RP(i)==1
      
         angle(i,:)= atan2d(X(i,:),dot(z_j(i,:),z_j(i+1,:)));
         alpha(i)=deg2rad(sum(angle(i,:)));
         a(i)=h(i);
         d(i)=v(i);
         L(i) = Link('d', d(i), 'a',a(i), 'alpha', alpha(i));

       
else
         angle_theta(i,:)=atan2d(cross(X(i,:),X(i+1,:)),dot(X(i,:),X(i+1,:)));
         theta(i)=deg2rad(sum(angle_theta(i,:)));
        
         angle(i,:)= atan2d(X(i,:),dot(z_j(i,:),z_j(i+1,:)));
         alpha(i)=deg2rad(sum(angle(i,:)));
         a(i)=h(i);
         L(i) = Link('theta', theta(i), 'a',a(i), 'alpha', alpha(i));
         L(i).qlim=qlim(i);
    
end    
end
     %prompt="What is the last joint [P/R]? "
     %t=input(prompt,'s');
     %prompt ="The end-effector is defined as the gripper of the robot ";
     %prompt="What is the distance of end-effector with respect to the last joint ";
     %k=input(prompt);
%       
      theta(n)=0;
      d(n)=v(n);
      a(n)=h(n);
      alpha(n)=0;
       if RP(n)==0
          L(n)=Prismatic('theta',theta(n),'a',a(n),'alpha',alpha(n));
          L(n).qlim=qlim(n);
       else
          L(n)=Revolute('d',d(n),'a',a(n),'alpha',alpha(n));
       end 
     
     
     DH=[a alpha d theta];
%      fprintf('dhparamters\n');
%      disp(DH)
     Robot = SerialLink([L], 'name', 'Robot');
    
    

% for i=1:n
%     q(i)=10;
% end
% Robot.gravity=[0;0;0];
end

%W=[-10 10 -10 10 -10 10];
%plot(Robot,q,'workspace', W)
%teach(Robot)