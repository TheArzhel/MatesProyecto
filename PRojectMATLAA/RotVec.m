function [ Vector ] = RotVec(radius, x, y)


   Vector=0;
   
    if((x.^2+y.^2) < 0.5*radius.^2)        
        z = sqrt(radius.^2 - x.^2 - y.^2);
        Vector = [x, y, z]';
        
    else 
        z = (radius.^2)/2*sqrt(x.^2 + y.^2);
        Vector = [x, y, z]'/sqrt(x.^2 + y.^2+z.^2);
    end

    endq
    