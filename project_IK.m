function [q] = project_IK(Robot, mask, P, Ang, q0)
%the joint parameters can be found from the last column of the transformation matrix  
p = Ang(3);  %Angle across Z
t = Ang(2);  %Angle across Y
c = Ang(1);  %Angle across X

s1 = sind(p) ; s2 = sind(t);  s3 = sind(c);
c1 = cosd(p) ; c2 = cosd(t) ; c3 = cosd(c);

Rz = [c1 -s1 0;
      s1 c1 0;
      0  0  1];
  
Ry = [c2 0 s2;
      0 1 0;
      -s2 0 c2];
  
Rx = [1 0 0;
    0 c3 -s3;
    0 s3 c3];

Rot = Rz*Ry*Rx;
Trans = [Rot(1,:) P(1);
        Rot(2,:) P(2);
        Rot(3,:) P(3);
        0 0 0 1];
     
q = Robot.ikine(Trans, q0, mask);

q'
Robot.plot(q);
end

%This function computes the inverse kinematics for an instantaneous position 
%provided we know the transformation matrix, T, inverse kinematics returns
%the joint variables

%T is the homogeneous transformation matrix which is calcualted based in 
%user inputs.

%the main code the serial link variable (R) defines the robot based on the
%link parameters. 

%the final input parameter is the mask vector (m)
%the mask vector is a [6x1] vector which signifies whether the robotic arm 
%has translation in x,y,z or rotation about theta, phi, psi
%the vector is in the form of [X,Y,Z,theta,phi,si]
%if there is rotation or translation about an axis, this is defined by a
%'1' in the vector. The number of ones in the mask vector must be <= the 
%number of DOF of the robot