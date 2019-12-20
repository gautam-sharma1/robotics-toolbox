function [E4,E5,E6] = project_inv_Euler_Angles(Rzyz,Rxyz,Rzyx,Rot)
% Need rotation matrix given by user
% Find Euler angles given rotation matrix
E4=[];
E5=[];
E6=[];
    if Rzyz == 1
    E4=tr2eul(Rot,'deg');  %this is only ZYZ
    fprintf('Euler angles are (sequential rotations about ZYZ)\n')
    disp(E4)
    end
    if Rxyz == 1
    E5 = tr2rpy(Rot,'deg'); %this is only XYZ
    fprintf('Euler angles are (sequential rotations about XYZ)\n')
    disp(E5)
    end
    if Rzyx == 1
    E6 = tr2rpy(Rot,'deg','zyx'); %this is only ZYX
    fprintf('Euler angles are (sequential rotations about ZYX)\n')
    disp(E6)
    end
end

