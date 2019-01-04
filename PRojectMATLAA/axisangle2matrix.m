function RotMat=axisangle2matrix(u_vector,Alpha_Angle)
%HELP
%This function return a rotation given a axis vector and an angle,
%the angles have to be in radiants.

I=eye(3);
  
u_vector=reshape(u_vector/norm(u_vector),[3,1]);
 

MatrixU=[0 -u_vector(3,1)  u_vector(2,1); 
         u_vector(3,1)  0  -u_vector(1,1);
         -u_vector(2,1) u_vector(1,1) 0];
    
RotMat=I*cos(Alpha_Angle)+((1-cos(Alpha_Angle))*((u_vector)*(u_vector)')+ MatrixU*sin(Alpha_Angle));

end