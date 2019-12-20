
function [J,dtJ] = project_singularity(Robot,z_j,RP,N)

syms q1 q2 q3 q4 q5 q6 q7 q8
thetad=[q1 q2 q3 q4 q5 q6 q7 q8];
n=N;

     L=Robot.links;
     %% Singularity part
     %% Calculating forward kinematrics for each link so to get the position vector to be used in Jacobian later
     for i=1:n
     R = SerialLink(L(1:i));
     g=thetad((1:i));
     p=R.fkine(g);
     p_a(:,i+1)=p(1:3,4);
     end
    p_a(:,1)=[0,0,0];

    %% Calculating Jacobian using standard using position vectors from
    z_j(n+1,:)=z_j(n,:);
for i=1:n

    if RP==1
        J(:,i)=[(cross((z_j(i,:)),(p_a(:,n+1)-p_a(:,i))'))';(z_j(i,:))'];

    else
        J(:,i)=[(z_j(i+1,:))';0;0;0];
    end
end

%% Shrinking the jacobian matrix to square so that we can take determinant.
%Using the fact that most of the times singularities occur due to the upper
%left corner sub-matrix
[row,col]=size(J);
J_s=J(1:col,1:col) %simplified J
  %det of simplified J

while det(J_s)==0
    J_s=J(1:col,1:col)
    dtJ = simplify(det(J_s));
    col=col-1;
end
    fprintf('Jacobian \n')
    disp(J)
%     fprintf('Jacobian simplified \n')
    disp(J_s)
    fprintf('Determinant of symbolic Jacobian \n')
    disp(dtJ)
    fprintf('Singularities are found by solving the symbolic Jacobian = 0 ')
end



%W=[-10 10 -10 10 -10 10];
%plot(Robot,q,'workspace', W)
%teach(Robot)
