function parametros_robo()

global mw R mc d L m Iw Ic I robo_dnm Ktorque Atrito Res Kp Ki sat;
global Fs Fk atr_s atr_k precisao;
% Parametros para simulação

% Massa (em gramas)             Fonte: https://www.masterwalkershop.com.br/kit-chassi-2wd-robo-robotica
% Chassi_completo = 450
% Roda_boba       = 32
% Roda            = 42 *2
% Caixa_de_Reducao= 60 *2       %(Inclui Motor e encoder) estimado
% Suporte_pilha   = 14
% Pilhas          = 40 *4
% Arduino         = 150
% Raspberry       = 150


% Massa de cada roda + motor
mw = 0.102;
%Raio da Roda
R = 0.034;
% Massa da plataforma 
mc = 1.3;
% Distancia do eixo até o centro de massa
d = 0;
% Distância entre as rodas
L = 0.147;
% Diâmetro da plataforma
D = 0.175*2;


% Massa total do robô
m = mc + 2*mw;

% Iw: Momento de inércia roda + motor
Iw = 0.5*mw*R^2;

%Momento de inércia da plataforma
Ic = (1/8)*mc*D^2;

% I: Momento de inércia total,
I = Ic + mc*d^2 + 2*mw*L^2;

%Atuadores
Ktorque = 10.913e-3;
Atrito  = 0.015;
Res     = 1.5506;

%PI
Kp = 30;
Ki = 15;

% Saturação
sat = 22;

% Zona Morta 
vel_min = 1.5;
zm = 5    ;

% Modelo dinâmico
% Constantes
M1 = Iw + (R^2/(4*L^2))*(m*L^2 + I);
M2 = (R^2/(4*L^2))*(m*L^2-Ic-2*mw*L^2);

% Modelo dinâmico
        % M(q)N_dot + V(q,q_dot)N + B(q)tau 
        % Saida = N = [Wd We]
M = [M1 M2; M2 M1];
V = zeros(2); % o centro de massa será considerado no centro do eixo das rodas
B = eye(2);
C = eye(2);
D = zeros(2);

%Define o modelo explicito
robo_exp = dss(V, B, C, D, M);
robo_dnm = ss(robo_exp,'explicit');

% Atrito
Fs = 0.025;
Fk = 0.020;
atr_s = 1;
atr_k = 1;

% Erro de posiçao aceitável
precisao = 0.1;
                                                                                % Constantes do modelo
                                                                               %A = (m*R^2/4+((I + m*d^2)*R^2/L^2) +Iw);
                                                                                %B = (m*R^2/4-((I + m*d^2)*R^2/L^2));
end

