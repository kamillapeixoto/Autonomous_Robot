function dydt = cinematico(t,y, wd, we)

global R L ;

% Estados

% 1 -  x
% 2 -  y
% 3 - theta

dydt = zeros(3,1);

%Modelo Cinemático
% x
dydt(1) = (R/2)*(cos(y(3))*wd + cos(y(3))*we);
% y
dydt(2) = (R/2)*(sin(y(3))*wd + sin(y(3))*we);
% theta
dydt(3) = (R/2)*(wd/L - we/L);

end

