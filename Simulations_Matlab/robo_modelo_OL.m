function dydt = robo_modelo_OL(t,y2, u1)


global robo_dnm Ktorque Res Atrito Kp Ki R L sat vel_min zm;
global Fs Fk atr_s atr_k;

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
erro      = u1(:,1) - y2(4:5) ;  % Fecha a malha de controle de velocidade
dydt(6:7) = erro;
ypid = Ki*y2(6:7)+Kp*erro;



Torque(1) = Ktorque*ypid(1)/Res -(Ktorque^2/Res + Atrito)*y2(4) - Fs*tanh(atr_s*y2(4)) + Fk*tanh(atr_k*y2(4)); 
Torque(2) = Ktorque*ypid(2)/Res -(Ktorque^2/Res + Atrito)*y2(5) - Fs*tanh(atr_s*y2(5)) + Fk*tanh(atr_k*y2(5));

 


%Saturaçao
%ypid_out =
%(1+sign(sat-abs(ypid_zm)))*ypid_zm/2+(1+sign(sat-abs(ypid_zm)))*sat*sign(ypid_zm)/2;


% Modelo Dinamico - Equações diferenciais
dydt(4:5) =  robo_dnm.A*y2(4:5) + robo_dnm.B*Torque;

% % Zona Morta
% 
% ypid_zm = ypid_in - zm*sign(ypid_in)-((1+sign(zm-abs(ypid_in)))/2).*(ypid_in-zm*sign(ypid_in));
% 
% dydt(4:5) = ypid_zm;

%Modelo Cinemático
% x
dydt(1) = (R/2)*(cos(y2(3))*y2(4) + cos(y2(3))*y2(5));
% y
dydt(2) = (R/2)*(sin(y2(3))*y2(4) + sin(y2(3))*y2(5));
% theta
dydt(3) = (R/2)*(y2(4)/L - y2(5)/L);

end

