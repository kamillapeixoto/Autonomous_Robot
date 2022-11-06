function dydt = robo_LQT(t,y, u1)

global robo_dnm Ktorque Res Atrito Kp Ki R L sat;
global Fs Fk atr_s atr_k;
global tensao;

%global tensao_tot tempo_tot;


% Estados
% 1 - x
% 2 - y
% 3- theta
% 4 - wd   - velocidade angular roda direita
% 5 - we   - velocidade angular roda esquerda
% 6 - erro  - erro de velocidade do motor direito
% 7 - erro  - erro de velocidade do motor esquerdo


dydt   = zeros(7,1);
Torque = zeros(2,1);

% PIs
erro      = u1(:,1) - y(4:5) ;  % Fecha a malha de controle de velocidade
dydt(6:7) = erro;
ypid = Ki*y(6:7)+Kp*erro;


ypid_out = ypid;

%Saturação do motor
if ypid_out(1) > sat
    ypid_out(1) = sat;
end   
if ypid_out(1) < -sat
    ypid_out(1) = -sat;
end   

if ypid_out(2) > sat
    ypid_out(2) = sat;
end

if ypid_out(2) < -sat
    ypid_out(2) = -sat;
end


tensao = [tensao ypid_out];
%tempo_tot =  [tempo_tot; t];

Torque(1) = Ktorque*ypid_out(1)/Res -(Ktorque^2/Res + Atrito)*y(4) - ...
            Fs*tanh(atr_s*y(4)) + Fk*tanh(atr_k*y(4)); 
Torque(2) = Ktorque*ypid_out(2)/Res -(Ktorque^2/Res + Atrito)*y(5) -...
            Fs*tanh(atr_s*y(5)) + Fk*tanh(atr_k*y(5));

% Modelo Dinamico - Equações diferenciais
dydt(4:5) = robo_dnm.A*y(4:5) + robo_dnm.B*Torque;

%Modelo Cinemático
% x
dydt(1) = (R/2)*(cos(y(3))*y(4) + cos(y(3))*y(5));
% y
dydt(2) = (R/2)*(sin(y(3))*y(4) + sin(y(3))*y(5));
% theta
dydt(3) = (R/2)*(y(4)/L - y(5)/L);
end

