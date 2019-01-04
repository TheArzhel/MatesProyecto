function [EulerAxis,Angle]=RotMatToEulerAxis_Angle(RotMat)
%INPUT:RotMat
%OUTPUT:EulerAxis,Angle

EulerAxis=[0 0 0];
Angle=0;

if det(RotMat)<1.0001 && det(RotMat)>0.9

Angle=acos((trace(RotMat)-1)/2);
if Angle ~= 0

CrossPMAtrix=(RotMat-RotMat')/(2*sin(Angle));
% CrossPMAtrix=[0           -U(3,1)  	(2,1); 
%               U(3,1)      0            -U(1,1);
%               -U(2,1)     U(1,1)          0];
% 

EulerAxis=[CrossPMAtrix(3,2) CrossPMAtrix(1,3) CrossPMAtrix(2,1)];
disp("Success");
else 
   disp("Angle equal zero")
   disp("Not possible to compute")
    
end
else 
   
    disp("determinant of RotMAt not equal to one"); 
    
end

disp("EulerAxis is:");
disp(EulerAxis);
disp("Angle is:");
disp(Angle);


end