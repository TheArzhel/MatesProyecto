function Rot_Mat=EulerAnglesToRotMat(roll,pitch,yaw )
% %INPUT:Pitch,Roll,Yaw
% %OUTPUT:RotMat

Rot_Mat=[cos(pitch)*cos(yaw) cos(yaw)*sin(pitch)*sin(roll)-cos(roll)*sin(yaw) cos(yaw)*cos(roll)*sin(pitch)+sin(yaw)*sin(roll);
                cos(pitch)*sin(yaw) sin(yaw)*sin(pitch)*sin(roll)+cos(roll)*cos(yaw) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
                -sin(pitch) cos(pitch)*sin(roll) cos(pitch)*cos(roll)];



% 
% disp("The rotation matrix is:");
% disp(RotMat);


end