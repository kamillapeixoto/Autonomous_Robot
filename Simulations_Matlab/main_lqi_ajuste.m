%% Malha fechada com LQI
global tensao_tot tempo_tot precisao;

contro    = []; % Verificar a controlabilidade do modelo OL
saida_li  = []; % Resultado da simulação linear
saida_nl  = []; % Resultado da simulação linear
saida_li2 = [];

%Estados iniciais
est = 0.001*ones(1,9);
est(1:2) = [0.8 0.8];
est(3) = 0;


estados = est;
y_lqi = est;
y_lqi2 = est';

tempo = 0:Tpid:Tloop;
%tempo = [0 Tpid*10];

u1 = ones(1,length(tempo));
u2 = ones(1,length(tempo));
u = 1*[u1;u2];

Q_opt = 1*eye(9);
R_opt = 1*eye(2);

y_out = est;
ref = 0;
iteracoes = [];
esf_tot = [];
tensao_tot =[];
erro_tot = [];

% Testar os valores de Q e R que levam ao menor tempo possível
for j=0.0001:0.1:1000001

    esforco_d = 0;
    esforco_e = 0;

% Inicia a simulação
while (dist(estados(end,1:2),u(:,1)') > precisao)
  %for ref =1:500   
    
   % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(3),estados(4), estados(5));

      % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
             
%  
    
    % Calcula os ganhos do LQT
    [K_lqi, modelo_cl] = lqi_CL(robo_sim,j*Q_opt,R_opt); 
        
    for i =1:14
        esf = -K_lqi*y_out(end,:)';
        
        
        tensao_tot = [];
        tempo_tot = [];
     erro_soma = erro_soma + sqrt(sum((u(:,1) - y_out(end,1:2)').^2));
        
        [t_out, y_out] = ode45(@(t,y) robo_LQI2(t,y, u, esf),[0 Tpid], estados);
        saida_nl = [saida_nl; y_out];
        estados = y_out(end,:);
         tensao_soma = tensao_soma + ...
                      sum(sum(abs(tensao_tot(:,1:end-1)).*diff(tempo_tot)'));
        
        


    end
%     

      ref= ref+1;
      
end

          j          
          estados = est;
ref
iteracoes = [iteracoes ref];
      
esf_tot  = [esf_tot; tensao_soma];
erro_tot = [erro_tot; erro_soma*Tpid];
esf_tot = [esf_tot; esforco_d*Tpid esforco_e*Tpid];
ref = 0;
end


figure()
plot(1:100:10001,[iteracoes'],'.k','MarkerSize',10)
xlim([0 10002])
ylim([0 210])
ylabel('Iterações')
xlabel('Peso Matriz Q')

figure()
plot(1:100:10001,esf_tot,'.k','MarkerSize',10)
xlim([0 10002])
ylabel('Esforço de Controle')
xlabel('Peso Matriz Q')

figure()
plot(1:100:10001,erro_tot,'.k','MarkerSize',10)
xlim([0 10002])
ylabel('Erro (m)')
xlabel('Peso Matriz Q')

minimo = min(iteracoes);
id = (iteracoes == minimo);
j=1:100:1000;
plot(j,iteracoes, '*k','LineWidth',1)
ylabel('Iterações')
xlabel('Peso Q')
