% Rotation and translation of vector
function [vect_n]= project_TransFrame(vect,angle,pos,axis) %takes input of rotation matrix



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

 vect_n = pos+R*vect;
 fprintf('Vector from base to the origin of new frame=\n')
 disp(vect_n)

 b = [0 0 0]';
axis_start = b;
R_0 = [1 0 0;
    0 1 0;
    0 0 1];

figure()

for i=1:3
    axis_end(:,i) = axis_start + R_0(:,i);
end

p=vect_n;
plot3(p(1), p(2), p(3), 'o');
grid on
hold on
for i = 1:3
    h=plot3([axis_start(1) axis_end(1,i)],...
        [axis_start(2) axis_end(2,i)],...
        [axis_start(3) axis_end(3,i)]);
    if i==1
        h.Color='red';
    elseif i==2
        h.Color='green';
    else
        h.Color='blue';
    end
end




axis_start = p;


for i=1:3
    axis_end(:,i) = axis_start + R(:,i);
end


plot3(b(1), b(2), b(3), 'o');

for i = 1:3
    h=plot3([axis_start(1) axis_end(1,i)],...
        [axis_start(2) axis_end(2,i)],...
        [axis_start(3) axis_end(3,i)]);
    if i==1
        h.Color='red';
    elseif i==2
        h.Color='green';
    else
        h.Color='blue';
    end
end
 plot3(p(1),p(2),p(3))
 xlabel('X'); ylabel('Y');zlabel('Z');
end
