function [T,A] = project_FK(a,alpha,d,theta,N,Robot,q0,DH)
% % Needed inputs: DH parameters (q is theta)
% % DH parameters can be found in dh_looks_good.m
% % Outputs: Robot transformation matrices (T), animation of transformations
% 
% % Make robotic arm
% % for i=1:size(a,2)
% % L(i)=Link('d',d(i),'a',a(i),'alpha', alpha(i), 'modified');
% % end

% for i = 1:n-1
%     type = RP(i);
% %     %create an if statement to establish the joint variables based on
% %     %whether they are prismatic or revolute
%     if type == 1
%         L(i)=Revolute('d',d(i),'a',a(i),'alpha', alpha(i));
%     else
%         L(i)=Prismatic('theta',theta(i),'a',a(i),'alpha', alpha(i));
%         L(i).qlim=qlim(i);
%     end
% end
%       theta(n)=0;
%       d(n)=0;
%       a(n)=a(n-1);
%       alpha(n)=0;
%        if RP(n)==0
%           L(n)=Prismatic('theta',theta(n),'a',a(n),'alpha',alpha(n));
%           L(n).qlim=qlim(n);
%        else
%           L(n)=Revolute('d',d(n),'a',a(n),'alpha',alpha(n));
%        end 
% Robot=SerialLink(L);
fprintf('Denavit Hartenberg parametrs (a,alpha,d,theta): \n');
disp(DH)

n=N;
for i=1:n
    A(:,:,i)=[cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i))  a(i)*cos(theta(i));
              sin(theta(i))  cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
              0              sin(alpha(i))               cos(alpha(i))                d(i);
              0              0                           0                            1];
   fprintf('T %i to %i = \n',i-1,i);
   disp(A(:,:,i))
end

% %Solve Transformation Matrices
% %T=Robot.fkine(q0,'deg');
T=Robot.fkine(q0);

fprintf('T 0 to E = \n');
disp(T)
% %animate transformations
% %for i=1:size(a,2)
figure
tranimate(T)
% %end

end

