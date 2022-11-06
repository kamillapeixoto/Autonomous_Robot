%% Malha fechada com LQT
global tensao  precisao;

contro    = []; % Verificar a controlabilidade do modelo OL

%Estados iniciais
est      = 0.001*ones(1,7);
est(1:2) = [0.8 0.8];
est(3)   = 0;

estados = est;
y_out   = est;


tempo = 0:Tpid:Tloop;
u     = ones(2,length(tempo));
 
Q_opt = eye(2);
R_opt = eye(2);

iteracoes     = 0;
iteracoes_tot = [];
esf_tot       = [];
tensao        = [];
erro_tot      = [];
u1_tot        = [];

peso_vet = 1:100:5001;

% Testar os valores de Q e R que levam ao menor tempo possível
for  peso =  peso_vet
        
    tensao_soma = 0;
    erro_soma   = 0;
    u1_soma     = 0;
    
% Inicia a simulação
while (dist(estados(end,1:2),u(:,1)') > precisao)
    
   % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(3),estados(4), estados(5));

      % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
             
 
    % Calcula os ganhos do LQT
    [K_lqt, F_lqt, modelo_cl] = lqt_CL(robo_sim,Q_opt*peso,R_opt); 
            
    
   for i =1:14
        tensao = [];
  
        u1 = F_lqt*u(:,1) - K_lqt*y_out(end,:)'; 
                        
        [t_out, y_out] = ode45(@(t,y) robo_LQT(t,y, u1),[0 Tpid], estados);
        estados = y_out(end,:);
        
        
        u1_soma = u1_soma + sum(abs(u1));
        
        erro_soma = erro_soma + sqrt(sum((u(:,1) - y_out(end,1:2)').^2));
        
        tensao_soma = tensao_soma + sum(abs(tensao(:,end)));
        
       % tensao_soma = tensao_soma + ...
       %               sum(sum(abs(tensao_tot(:,1:end-1)).*diff(tempo_tot)'));
        
   end
    % Atualiza as condições iniciais
      estados = y_out(end,:);
    
      iteracoes= iteracoes+1;
end
          peso          
          estados = est;
iteracoes
iteracoes_tot = [iteracoes_tot iteracoes];
iteracoes = 0;

        
%esf_tot  = [esf_tot; tensao_soma];
esf_tot  = [esf_tot ; tensao_soma*Tpid];
erro_tot = [erro_tot; erro_soma*Tpid];
u1_tot   = [u1_tot; u1_soma*Tpid];

end

plot_ajuste(iteracoes_tot, esf_tot, erro_tot, u1_tot, peso_vet)