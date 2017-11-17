function Jv = computeJacobian(theta1, theta2, theta3, a1, a2, a3)

% T0_1 = [ cos(theta1)  0 -sin(theta1)   0           ;...
%          sin(theta1)  0  cos(theta1)   0            ;...
%               0       -1      0        a1            ;...
%               0       0       0        1            ];
% T1_2 = [ sin(theta2)  cos(theta2)  0  a2*sin(theta2);...
%         -cos(theta2)  sin(theta2)  0 -a2*cos(theta2);...
%               0            0       1       0             ;...
%               0            0       0       1             ];
% T2_3 = [-sin(theta3)  -cos(theta2)  0 -a3*sin(theta3);...
%          cos(theta3)  -sin(theta2)  0  a3*cos(theta3);...
%               0             0       1       0             ;...
%               0             0       0       1             ];
% T0_3 = T0_1 * T1_2 * T2_3;
% o0_3 = T0_3 * [0 0 0 1]';
% 
% x = o0_3(1);
% y = o0_3(2);
% z = o0_3(3);
% 
% Jv(:,1) = diff([x;y;z], theta1);
% Jv(:,2) = diff([x;y;z], theta2);
% Jv(:,3) = diff([x;y;z], theta3);
%

% Alternate
Jv = [ -sin(theta1)*(a3*cos(theta2 + theta3) + a2*sin(theta2)), -cos(theta1)*(a3*sin(theta2 + theta3) - a2*cos(theta2)), -a3*sin(theta2 + theta3)*cos(theta1);
     cos(theta1)*(a3*cos(theta2 + theta3) + a2*sin(theta2)), -sin(theta1)*(a3*sin(theta2 + theta3) - a2*cos(theta2)), -a3*sin(theta2 + theta3)*sin(theta1);
                                             0,              - a3*cos(theta2 + theta3) - a2*sin(theta2),             -a3*cos(theta2 + theta3)];

end