%% Manipulator dynamics 
function [q,qd] = project_FD(a,alpha,d,theta,m_li,m_mo,I_li,I_mo,kr,G,Tau,q0,qd0,qlim,RP,N,tspan,frame)

%code takes in DH parameters and defines the robot including the masses and
%moment of inertia
%% Test cases
% q0 = [0.7 2 0.5];
% qd0 = [0.7 2 0.5];
% Tau = [20,5,1]; %constant torques applied to each joint
% a = [1 1 1]; %length of the link
% alpha = [0 0 0]; 
% d = [0 0 0]; %
% theta = [30 40 50];
% m_li = [10 10 10]; %mass of the link
% I_li = [10 10 10];  %inertia of the link
% I_mo = [0.05 0.05 0.05]; %inertia of the motor 
% kr = [90 90 90]; %gear ratio
% RP = [1 1 1]; %tells whether the joint is revolute or prismatic
% G = [0 1 0];
% N = 3;
% tspan = 10;
%% 
%qdot0 = zeros(1,N); %the robot will start from rest, so the velocities will start from 0 

%find the links to then extract the rotation matrices and apply them to the
%link calculations which will be used in fdyn
%%
%find the links to then extract the rotation matrices and apply them to the
%link calculations which will be used in fdyn
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
    
%define the rotation matrix 
%if the robot is defined in the base frame, we need to consider the
%rotation matrix. if it is defined in the local frame, then the rotation
%matrix is just the identity (ie: R(i) = eye(3))
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



%define the link parameters 
%type determines whether the link is revolute or prismatic
for i = 1:N-1
    type = RP(i);
    %create an if statement to establish the joint variables based on
    %whether they are prismatic or revolute
    if type == 1
        L(i) = Revolute('d',d(i),'a',a(i),'alpha',alpha(i),'m',m_li(i)+m_mo(i+1),...
            'I',Rli{i}'*I_li(i)*Rli{i},'G',kr(i),'Jm',Rm{i}'*I_mo(i)*Rm{i});
    else
        L(i) = Prismatic('theta',theta(i),'a',a(i),'alpha',alpha(i),...
            'm',m_li(i)+m_mo(i+1),'I',Rli{i}'*I_li(i)*Rli{i},'G',kr(i),'Jm',Rm{i}'*I_mo(i)*Rm{i});
        L(i).qlim=qlim(i);
    end
end

    if RP(N) == 1
        L(N) = Revolute('d',d(N),'a',a(N),'alpha',alpha(N),'m',m_li(N),...
            'I',Rli{N}'*I_li(N)*Rli{N},'G',kr(N),'Jm',Rm{N}'*I_mo(N)*Rm{N});
    else
        L(N) = Prismatic('theta',theta(N),'a',a(N),'alpha',alpha(N),'m',m_li(N),...
            'I',Rli{N}'*I_li(N)*Rli{N},'G',kr(N),'Jm',Rm{N}'*I_mo(N)*Rm{N});
        L(N).qlim=qlim(N);
    end
    
robot = SerialLink(L);
robot.gravity = 9.81*G; %specify to the robot which direction gravity acts in 

%solve the forward dynamics problem
%output: the joint angle and rate vs time, each in its own vector
%the time is defined at 15 sec
[t,q,qd] = robot.fdyn(tspan,@Trq,q0,qd0,Tau'); %use the fdyn function in the robotics toolbox, specifying the initial joint position/velocity, the torque applied, and the time interval

%% plot of joint angles vs time
[r,c] = size(qd); 
for i = 1:c
    li = i;
    Legend1{i} = strcat('q',num2str(li));
end

for i = 1:c
    li = i;
    Legend2{i} = strcat('qd',num2str(li));
end

figure
plot(t,q)
xlabel('Time (sec)'); ylabel('Joint');
title('Joint Value vs Time'); grid on; legend(Legend1);

%three joint rates vs time
figure
plot(t,qd)
xlabel('Time (sec)'); ylabel('Joint Velocity');
title('Joint Velocity vs Time'); grid on; legend(Legend2);

%function handle for fdyn
%function handle expects the initial torqe and joint parameters regardless
%if torque is constant or not
function T = Trq(t,q,qd,a,Tau)
    T = Tau;
end

end