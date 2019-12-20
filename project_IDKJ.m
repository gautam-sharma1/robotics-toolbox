    %Inverse differential kinematics
    %assuming Robot 'R' is defined
function  [qdot]= project_IDKJ(Robot,q0,ve) 
  

    j0 = Robot.jacob0(q0);  %q is 1XN array of pose joint variables
    
    %prompt ="Please enter the end effector velocities in the format of [vx vy vz wx wy wz] ";
   
    %ve = input(prompt);
    %vx=ve(1); vy=ve(2); vz=ve(3); wx=ve(4); wy=ve(5); wz=ve(6);
    
    [m,n]=size(j0);
    %k=rank(j0);
    if m~=n || det(j0)==0
        %disp("Matrix is not invertible")
        qdot=pinv(j0)*ve';
    else
       
        qdot=inv(j0)*ve';
        
    end
       
    qdot
    end
    
    
    