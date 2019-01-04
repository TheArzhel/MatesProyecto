function [ Quat ] = TwoVec_To_Quat(Axis,Vec )

    Aux = sqrt(2+2*dot(Axis,Vec));
   Aux2 = (1/m)*cross(Axis,Vec);
   Quat = [Aux/2;Aux2];
   
end