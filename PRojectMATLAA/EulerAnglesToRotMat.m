function RotMat=EulerAnglesToRotMat(Roll,Pitch,Yaw)
%INPUT:Pitch,Roll,Yaw
%OUTPUT:RotMat

alpha=(cos(Pitch)*cos(Yaw));

beta=(cos(Yaw)*sin(Pitch)*sin(Roll)-cos(Roll)*sin(Yaw));

gamma=(cos(Yaw)*cos(Yaw)*sin(Pitch)+sin(Yaw)*sin(Roll));

delta= (cos(Pitch)*sin(Yaw));

epsilon= (sin(Yaw)*sin(Pitch)*sin(Roll)+cos(Roll)*cos(Yaw));

fi= (sin(Yaw)*sin(Yaw)*cos(Pitch)-cos(Yaw)*sin(Roll));

psi=-sin(Pitch);

tau= cos(Pitch)*sin(Roll);

eta=  cos(Pitch)*cos(Roll);

RotMat = [alpha beta gamma; delta epsilon fi; psi tau eta];


disp("The rotation matrix is:");
disp(RotMat);


end