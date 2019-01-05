function RotMat=QuatToRotMat(Quat)
%INPUT:Quat
%OUTPUT:RotVec
  
QuatNorm=sqrt(Quat(1)^2+Quat(2)^2+Quat(3)^2+Quat(4)^2);
%unit quaternion
Versor=Quat/QuatNorm;
Quat=Versor;

QuatVec=Quat(2:length(Quat));
QuatVec = QuatVec/norm(QuatVec);

QuatVec = reshape(QuatVec,[3,1]);

QMatP=[0 -QuatVec(3,1) QuatVec(2,1);QuatVec(3,1) 0 -QuatVec(1,1); -QuatVec(2,1) QuatVec(1,1) 0];

a= Quat(1)^2 + Quat(2)^2-Quat(3)^2-Quat(4)^2;

b= 2*Quat(2)*Quat(3)- 2*Quat(1)*Quat(4);

c=2*Quat(2)*Quat(4)+2*Quat(1)*Quat(3);

d= 2*Quat(2)*Quat(3)+2*Quat(1)*Quat(4);

e= Quat(1)^2-Quat(2)^2+Quat(3)^2-Quat(4)^2;

f=2*Quat(3)*Quat(4)-2*Quat(1)*Quat(2);

g=2*Quat(2)*Quat(4)-2*Quat(1)*Quat(3);

h=2*Quat(3)*Quat(4)+2*Quat(1)*Quat(2);

i=Quat(1)^2-Quat(2)^2-Quat(3)^2+Quat(4)^2;

disp("The RotMatrix is:")

RotMat = [a b c; d e f; g h i]




%%RotMat=((Quat(1))^2-QuatVec'*QuatVec)*eye(3,3)+2*(QuatVec*QuatVec)+ 2*Quat(1)*QMatP ;

end
