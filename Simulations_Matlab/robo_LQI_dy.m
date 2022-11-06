function dydt = robo_LQI_dy(u_lqi, K_lqi)

global robo_dnm Ktorque Res Atrito Kp Ki R L sat y_lqi2;

% Estados
% 1 - x
% 2 - y
% 3- theta
% 4 - wd
% 5 - we
% 6 - erro  - erro de velocidade do motor direito
% 7 - erro  - erro de velocidade do motor esquerdo
% 6 - x estendido
% 7 - y estendido


dydt = zeros(9,1);

Torque = [0 ;0];

% Controlador de trajetória
errotraj  = u_lqi(:,1) - y_lqi2(1:2);
dydt(8:9) = errotraj;
u_lqi = -K_lqi*y_lqi2;          

% PIs
erro      = u_lqi - y_lqi2(4:5);   % Fecha a malha de controle de velocidade
dydt(6:7) = erro;
y_pid = Ki*y_lqi2(6:7)'+Kp*erro;

%Saturação do motor
if y_pid(1) > sat
    y_pid(1) = sat;
end   
if y_pid(1) < -sat
    y_pid(1) = -sat;
end   

if y_pid(2) > sat
    y_pid(2) = sat;
end

if y_pid(2) < -sat
    y_pid(2) = -sat;
end

%Modelo dos atuadores
Torque(1) = Ktorque*y_pid(1)/Res -(Ktorque^2/Res + Atrito)*y_lqi2(4); 
Torque(2) = Ktorque*y_pid(2)/Res -(Ktorque^2/Res + Atrito)*y_lqi2(5); 

% Modelo Dinamico - Equações diferenciais
dydt(4:5) = robo_dnm.A*y_lqi2(4:5) + robo_dnm.B*Torque;

%Modelo Cinemático
% x
dydt(1) = (R/2)*(cos(y_lqi2(3))*y_lqi2(4) + cos(y_lqi2(3))*y_lqi2(5));

% y
dydt(2)  = (R/2)*(sin(y_lqi2(3))*y_lqi2(4) + sin(y_lqi2(3))*y_lqi2(5));

% theta
dydt(3)  = (R/2)*(y_lqi2(4)/L - y_lqi2(5)/L);

end

