%%This function takes in a vector v containing the a, alpha, distance,
% and the theta. And it returns a transformation A. 
function A = transformation(v)
    a=v(1);
    alpha=v(2);
    dist=v(3);
    theta=v(4);
    RotZTheta = [cos(theta), -sin(theta), 0,0; 
                sin(theta), cos(theta), 0,0;
                0, 0, 1,0;
                0,0,0,1];
            
    TransZD = [1,0,0,0;
               0,1,0,0;
               0,0,1,dist;
               0,0,0,1];
    
    TransXA = [1,0,0,a;
               0,1,0,0;
               0,0,1,0;
               0,0,0,1];
    
           
    RotZAlpha = [1,0,0,0;
                 0,cos(alpha), -sin(alpha), 0;
                 0,sin(alpha), cos(alpha), 0;
                 0,0,0,1];
    
         
     A = (((RotZTheta * TransZD) * TransXA)  * RotZAlpha);
    
end

