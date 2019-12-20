%Motion_Control_compliance only_Kp and Kd
function [xr_history] = project_MotionControl(q0,qd0,a,alpha,d,m_li,m_mo,I_li,I_mo,qlim,kr,G,N,tspan,Kp,Kd,x_d,frame,RP)

T_S = 0.01;
Final_Time = tspan;
N_iteration = Final_Time/T_S;

K_P = Kp * diag([1 1 0]);   
K_D = Kd* diag([1 1 0]);
    
x_d=x_d';
x_d=x_d(1:3);

%--------------------creating a Local global based link-------------%
for i = 1:N-1
    type = RP(i);
    %create an if statement to establish the joint variables based on
    %whether they are prismatic or revolute
    if type == 1
        L(i) = Revolute('d',d(i),'a',a(i),'alpha',alpha(i));
    else
        L(i) = Prismatic('theta',theta(i),'a',a(i),'alpha',alpha(i));
    end
end

if RP(N) == 1
        L(N) = Revolute('d',d(N),'a',a(N),'alpha',alpha(N));
    else
        L(N) = Prismatic('theta',theta(N),'a',a(N),'alpha',alpha(N));
end

if frame == 1 %if the frame is in the base frame, find the rotation matrix
   for i = 1:N
       Li(i) = L(i);
       %q0(i) = q0(i,1);
       robot = SerialLink(Li);  %define the robot
       %J = robot.jacob0(q0); %compute jacobian 
       T = robot.fkine(q0);
       %Rli{i} = [T.n,T.o,T.a];
       Rli{i} = T(1:3,1:3);
   end
   
   Rm{1} = eye(3); 
   for i = 1:N-1
       Rm{i+1} = Rli{i};
   end
else %the frame is in the local frame
   for i = 1:N
       Rli{i} = eye(3);
       Rm{i} = eye(3);
   end
end
    
%--------------------------------------------------------------------------


for i=1:N
    if i==1
        L(i) = Revolute('d',d(i),'a',a(i),'alpha',alpha(i),'m',m_li(i)+m_mo(i+1),...
                    'I',Rli{i}'*I_li(i)*Rli{i},'G',kr(i),'Jm',Rm{i}'*I_mo(i)*Rm{i});
    elseif i==2    
        L(i) = Revolute('d',d(i),'a',a(i),'alpha',alpha(i),'m',m_li(i),...
                    'I',Rli{i}'*I_li(i)*Rli{i},'G',kr(i),'Jm',Rm{i}'*I_mo(i)*Rm{i});
    end 
end

robot = SerialLink(L);

robot.gravity = 0*G; 

if q0(1) > qlim(1,2) || q0(1) < qlim(1,1) || q0(2) > qlim(2,2) || q0(2) < qlim(2,1) 
    fprintf('Robot joints initials are not defined in the limits intervals')
end


q_i = q0;
qd_i = qd0;
xr = ForwardKinematic(robot,q_i);

for i = 1:N_iteration+1

q_history(:,i) = q_i;
qd_history(:,i) = qd_i;
xr_history(:,i) = xr;

xr = ForwardKinematic(robot,q_i);
xe = x_d - xr;
u_kp = K_P * xe;

xd = MyJacobian(robot,q_i)*qd_i;
xd = xd(1:3);
u_kd = K_D * xd;
% u_g = gravityeffect(a,alpha,d,q_i,m_li,m_mo,I_li,I_mo,kr); %equal to zero
Jac_Inv = MyJacobian_Transpose(robot,q_i);
Jac_Inv = Jac_Inv(:,1:3);
u_controller = Jac_Inv*(u_kp-u_kd);% + u_g';

[q,qd] = Manipulator_Dynamics(robot,T_S,u_controller',q_i,qd_i);
q_i = q(end,:)';
qd_i = qd(end,:)';

end


%creating time for plot        
T=0:T_S:Final_Time;
%creating constant x_d for plot
x_dp=transpose(ones(2,N_iteration+1).*x_d(1:2)); 

%defining legend
Legend{1} = strcat('x axis');
Legend{2} = strcat('x desired');
Legend{3} = strcat('y axis');
Legend{4} = strcat('y desired');

figure
plot(T,xr_history(1,:)','b')
hold on
plot(T,x_dp(:,1),'b--')
plot(T,xr_history(2,:)','r')
plot(T,x_dp(:,2),'r--')
xlabel('Time (sec)'); ylabel('End-effector Position (m)');grid on;
title('End-effector motion controlled by compliance PD controller'); legend(Legend);

end

function [q,qd] = Manipulator_Dynamics(Robot,t_s,Tau,q,qd)
 
[t,q,qd] = Robot.fdyn(t_s,@Trq,q,qd,Tau); 

function T = Trq(t,q,qd,a,Tau)
    T = Tau;
end

end

function  [J_T]= MyJacobian_Transpose(Robot,q) 
  
    J = Robot.jacob0(q); 
    
    J_T=pinv(J);
    
end
    
function  [J]= MyJacobian(Robot,q0) 
     J = Robot.jacob0(q0);   
end

function [x] = ForwardKinematic(Robot,q)

T=Robot.fkine(q);
x=T(1:3,4);

end