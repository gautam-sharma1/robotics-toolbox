function [E1, E2, E3] = project_Euler_Angles(roll, pitch, yaw, Rzyz, Rxyz, Rzyx)
% Need Euler angles as user input
% Calculate Rotation matrix for Z-Y-Z Euler Angle
E1=[];
E2=[];
E3=[];
if(Rzyz ==1 )    
E1 = eul2r(roll, pitch, yaw,'deg','zyz');
fprintf('Rzyz=\n');
disp(E1);
end
% Calculate Rotation matrix for X-Y-Z Euler Angles
if(Rxyz ==1) 
T1 = rpy2tr(roll, pitch, yaw, 'deg','xyz');
E2 = T1(1:3,1:3);
fprintf('Rxyz=\n');
disp(E2);
end
% Calculate Rotation matrix for Z-Y-X Euler Angles
if(Rzyx ==1) 
T2 = rpy2tr(roll, pitch, yaw, 'deg','zyx');
E3 = T2(1:3,1:3);
fprintf('Rzyx=\n');
disp(E3);
end

if(Rzyz == 0 && Rxyz ==0 && Rzyx ==0)
    E1=0; E2 = 0; E3 = 0;
    disp('Please select a coodinate');
end
