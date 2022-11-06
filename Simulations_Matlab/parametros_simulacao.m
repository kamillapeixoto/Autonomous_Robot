function parametros_simulacao()
% Tempo (em s)

global Tpid Tloop Tsim Nloop;

% Periodo de amostragem do loop do PID
Tpid = 0.035;
% Periodo do loop do raspberry
Tloop = 0.48;
% Duração da simulação
Tsim = 100;
% Quantidade de loops a ser simulado
Nloop = 20;


end

