function Result=QuatMult(quat1,quat2)
%%q=quat1 p=quat2

quat1=reshape(quat1,[1 4]);
quat2=reshape(quat2,[1 4]);

Scalar1=quat1(1);
Scalar2=quat2(1);

VectorQuat1=reshape(quat1(2:length(quat1)),[1 3]);
VectorQuat2=reshape(quat2(2:length(quat2)),[1 3]);

var1=Scalar1*Scalar2;
var2=VectorQuat1*VectorQuat2';



Result=[var1-var2 ((Scalar1*VectorQuat2)+(Scalar2*VectorQuat1)+cross(VectorQuat1,VectorQuat2))]';

end