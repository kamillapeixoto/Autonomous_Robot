%% LQT e APF

global mw R mc d L m Iw Ic I robo_dnm Ktorque Atrito Res Kp Ki sat vel_min;
global Tpid Tloop Tsim Nloop;
global K_I y_lqi y_lqi2 y_lqt zm;
global Fs Fk atr_s atr_k precisao;
global tensao  precisao;
global etta xi ro_o a_max

parametros_robo();
parametros_simulacao();
parametros_APF();


%% Malha fechada com LQT


contro    = []; % Verificar a controlabilidade do modelo OL
saida_li2  = []; % Resultado da simulação linear
saida_nl  = []; % Resultado da simulação nao linear

%Estados iniciais
est     = 0.001*ones(1,7);
%est(3
estados = est;
y_out   = est;

pos_objetivo   = [2.5 2.5];
pos_obstaculos = [0.73,2.14;
                  1.48,1.44;
                  2.14,0.73];

tempo = 0:Tpid:Tloop;
u     = ones(2,length(tempo));

peso  = 4001;
Q_opt = peso*eye(2);
R_opt = 1*eye(2);

iteracoes = 0;

esf_tot       = [];
tensao_tot    = [];
erro_tot      = [];
u1_tot        = [];

erro_soma = 0;
tensao_soma = 0;

% Inicia a simulação
while (dist(estados(end,1:2),pos_objetivo(:,1)') > precisao)
  %for iteracoes =1:500   
    
   % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(3),estados(4), estados(5));

      % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
             
 
    % Calcula a força do APF
    F_apf      = APF(estados(1:2), pos_objetivo, pos_obstaculos);
    
    
    % Calcula os ganhos do LQT
    [K_lqt, F_lqt, modelo_cl] = lqt_CL(robo_sim,Q_opt,R_opt); 
    
   
        
   for i =1:13
       
        erro_soma = erro_soma + sqrt(sum((u(:,1) - y_out(end,1:2)').^2));
        
        
        u1 = F_lqt*F_apf' - K_lqt*y_out(end,:)'; 
    
        [t_out, y_out] = ode45(@(t,y) robo_LQT(t,y, u1),[0 Tpid], estados);
        saida_nl = [saida_nl; y_out];
        estados = y_out(end,:);
        
%        tensao_soma = tensao_soma + sum(abs(tensao_tot(:,end)));
%         
%         erro_tot   = [erro_tot  sqrt(sum((u(:,1) - y_out(end,1:2)').^2))];
%         
%         tensao_tot = [tensao_tot  tensao(:,end)];
%         
  
   end
   
        iteracoes = iteracoes+1
        
end

% u1_soma     = Tpid*sum(sum(abs(u1_tot)));
% tensao_soma = Tpid*sum(sum(abs(tensao_soma)));
% erro_soma   = Tpid*sum(erro_tot);


%%
temp = linspace(0, Tloop*iteracoes, length(saida_nl));
plotar_apf(saida_nl, est(1:2), pos_objetivo, pos_obstaculos, temp)
