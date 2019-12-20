function [T] = project_homogenous(angle,pos,axis)
T= eye(4);
n=0;

%while axis~=0

%axis= input("Enter the axis ",'axis');

switch axis
    case {'x','X'}
        R = [1 0 0 ; 0 cosd(angle) -sind(angle); 0 sind(angle) cosd(angle)];
    case {'y','Y'}
        R = [cosd(angle) 0 sind(angle); 0 1 0; -sind(angle) 0 cosd(angle)];
    case {'z','Z'}
        R = [cosd(angle) -sind(angle) 0; sind(angle) cosd(angle) 0; 0 0 1];
    otherwise
        disp('Unknown axis. Please use x, y or z');
        R = [];
end

T=T*[R pos; 0 0 0 1];

%end

T

end
