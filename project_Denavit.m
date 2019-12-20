function [DH,Robot] = project_Denavit(a,alpha,d,theta,RP,N,qlim)

n=N;
 
 for i=1:n
% prompt ="What is the "+i+"st joint [P/R] ";
% k=input(prompt,'s');


if RP(i)==1
           
         L(i) = Link('d', d(i), 'a',a(i), 'alpha', alpha(i));    
else
 
         L(i) = Link('theta', theta(i), 'a',a(i), 'alpha', alpha(i));
         L(i).qlim=qlim(i);
    
end
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