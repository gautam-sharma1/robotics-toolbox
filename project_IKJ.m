function [q]= project_IKJ(Robot,tspan,ve,q0)

dt=0.1;
t=0:dt:tspan;

% prompt ="Please enter the end effector velocities in the format of [vx vy vz wx wy wz] ";
%    
%     ve = input(prompt);
%     vx=ve(1); vy=ve(2); vz=ve(3); wx=ve(4); wy=ve(5); wz=ve(6);
% 


 for i=1:length(t)
          q(i,:)=q0;
          j0 = Robot.jacob0(q0); %Calculating jacobian
          [m,n]=size(j0);
         % k=rank(j0);
          if m~=n || det(j0)==0      %checking if Jacobian is full rank or det=0
              %disp("Matrix is not invertible");
              qd=pinv(j0)*ve';    %Can we assume ve remains constant?  don't need ve can be either constant or variable if it is variable you have to define it based on time ti so at each time it has a value          
          else
              qd=inv(j0)*ve';
          end
     
          q0=q0+qd*dt; %Algorithm that takes q0 to qd          
 end  
  figure()
  Robot.plot(q)
  q;
end

