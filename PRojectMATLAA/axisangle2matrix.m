function RotMat=axisangle2matrix(u_vector,Alpha_Angle)
%HELP
%This function return a rotation given a axis vector and an angle,
%the angles have to be in radiants.

if nargin<1
  
   u_vector=[1 0 0];
elseif nargin<2
   
 Alpha_Angle=pi/2;
 
else
    flag=0;
    
    if length(u_vector)~=3 
     disp("Dimension of the u_vector ");
     flag=1;
    end
    if Alpha_Angle>2*pi
        disp("Angle in radians");
      
    end
    
end

%Angle in Radiants

if flag == 0
%reshape(u_vector,[3,1]);
I=eye(3);



  
u_vector=reshape(u_vector/norm(u_vector),[3,1]);
 

MatrixU=[0 -u_vector(3,1)  u_vector(2,1); 
         u_vector(3,1)  0  -u_vector(1,1);
         -u_vector(2,1) u_vector(1,1) 0];
    
RotMat=I*cos(Alpha_Angle)+((1-cos(Alpha_Angle))*((u_vector)*(u_vector)')+ MatrixU*sin(Alpha_Angle));


end

return;

end