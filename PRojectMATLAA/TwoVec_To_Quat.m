function [ Quat ] = TwoVec_To_Quat(Vector1,Vector2 )

    Aux = sqrt(2+2*dot(Vector1,Vector2));
   Aux2 = (1/Aux)*cross(Vector1,Vector2);
   Quat = [Aux/2;Aux2];
   
end