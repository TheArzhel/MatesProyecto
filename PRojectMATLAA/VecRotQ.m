function NewVec=VecRotQ(Vec,Quat)

Vec=reshape(Vec,[1,3]);
Quat=reshape(Quat,[1,4]);

QuatNorm=sqrt(Quat(1)^2+Quat(2)^2+Quat(3)^2+Quat(4)^2);
%unit quaternion
Versor=Quat/QuatNorm;
Quat=Versor;

PureVec=[0 Vec];
QuatVec=Quat(2:length(Quat));
QuatConj=[Quat(1) -QuatVec];



QuatVec=QuatMult(QuatMult(Quat,PureVec),QuatConj);

NewVec=QuatVec(2:length(QuatVec));
NewVec=NewVec';

disp("Rotated Vector is");
disp(NewVec);

    
end