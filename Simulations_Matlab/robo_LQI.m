function dydt = robo_LQI2(t,y, referencia, u_lqi)

global robo_dnm Ktorque Res Atrito Kp Ki R L sat;
global Fs Fk atr_s atr_k;
global tensao_tot tempo_tot;


% Estados
% 1 - x
% 2 - y
% 3- theta
% 4 - wd
% 5 - we
% 6 - erro  - erro de velocidade do motor direito
% 7 - erro  - erro de velocidade do motor esquerdo
% 8 - x estendido
% 9 - y estendido


dydt = zeros(9,1);

Torque = [0 ;0];

      

% PIs
erro      = u_lqi - y(4:5);   % Fecha a malha de controle de velocidade
dydt(6:7) = erro;
ypid = Ki*y(6:7)+Kp*erro;

%Saturação do motor
if ypid(1) > sat
    ypid(1) = sat;
end   
if ypid(1) < -sat
    ypid(1) = -sat;
end   

if ypid(2) > sat
    ypid(2) = sat;
end

if ypid(2) < -sat
    ypid(2) = -sat;
end


tensao_tot = [tensao_tot ypid];
tempo_tot =  [tempo_tot; t];

%Modelo dos atuadores
Torque(1) = Ktorque*ypid(1)/Res -(Ktorque^2/Res + Atrito)*y(4) -...
            Fs*tanh(atr_s*y(4)) + Fk*tanh(atr_k*y(4)); 
Torque(2) = Ktorque*ypid(2)/Res -(Ktorque^2/Res + Atrito)*y(5)- ...
            Fs*tanh(atr_s*y(5)) + Fk*tanh(atr_k*y(5)); 

% Modelo Dinamico - Equações diferenciais
dydt(4:5) = robo_dnm.A*y(4:5) + robo_dnm.B*Torque;

%Modelo Cinemático
% x
dydt(1) = (R/2)*(cos(y(3))*y(4) + cos(y(3))*y(5));

% y
dydt(2)  = (R/2)*(sin(y(3))*y(4) + sin(y(3))*y(5));

% theta
dydt(3)  = (R/2)*(y(4)/L - y(5)/L);

% Controlador de trajetória
errotraj  = referencia(:,1) - y(1:2);
dydt(8:9) = errotraj;



end

