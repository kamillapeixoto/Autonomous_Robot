%% Malha fechada com LQT
global tensao  precisao;

contro    = []; % Verificar a controlabilidade do modelo OL
saida_li2  = []; % Resultado da simulação linear
saida_nl  = []; % Resultado da simulação nao linear

%Estados iniciais
est     = 0.001*ones(1,7);
estados = est;
y_out   = est;

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

pos_objetivo   = [2.5 2.5];
pos_obstaculos = [0.73,2.14;
                  1.48,1.44;
                  2.14,0.73];
              
              

% Inicia a simulação
while (dist(estados(end,1:2),pos_objetivo') > precisao)
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
    F_apf = APF(estados(1:2), pos_objetivo, pos_obstaculos);
    
    % Calcula o proximo setpoint de posição
    u = estados(1:2) + pol2rec(F_apf);
     
    % Calcula os ganhos do LQT
    [K_lqt, F_lqt, modelo_cl] = lqt_CL(robo_sim,Q_opt,R_opt); 
       
    
    
        
   for i =1:13
       
        erro_soma = erro_soma + sqrt(sum((u(:,1) - y_out(end,1:2)').^2));
        

        
        u1 = F_lqt*u(:,1) - K_lqt*y_out(end,:)'; 
    
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
figure()
plot(temp,saida_nl(:,4), 'k', 'LineWidth',3)
hold on
plot(temp, saida_nl(:,5), 'Color', [0.7 0.7 0.7],'LineWidth',3)
xlabel('Tempo (s)')
ylabel('Velocidade (rad/s)')
legend('Direita', 'Esquerda')
ylim([-10 14])


figure()
plot(temp,saida_nl(:,6), 'k', 'LineWidth',3)
hold on
plot(temp, saida_nl(:,7), 'Color', [0.7 0.7 0.7],'LineWidth',3)
xlabel('Tempo (s)')
ylabel('X_{PI} (rad/s)')
legend('Direita', 'Esquerda')


plot(temp,wrapTo360(rad2deg(saida_nl(:,3))), 'k', 'LineWidth',3)
xlabel('Tempo(s)')
ylabel('\theta (º)')

legend('Velocidade direita','Velocidade esquerda', 'theta')

figure()
plot(saida_nl(:,1), saida_nl(:,2), 'k', 'LineWidth',3)
title('Trajetória')
xlabel('x (m)')
ylabel('y (m)')
hold on
plot(1,1,'xk')
xcir = 1;
ycir = 1;
radius = 0.1;
ang=0:0.01:2*pi;
xp=radius*cos(ang);
yp=radius*sin(ang);
h4 = plot(xcir+xp,ycir+yp, '-.k');
% xlim([0.75 1.05])
% ylim([0.75 1.05])
legend('Percurso','Posição Objetivo','Círculo de Precisão')
xlim([-0.05 1.15])
ylim([-0.05 1.15])

figure()
plot(saida_nl(:,1))
figure()
plot(saida_nl(:,2))
figure()


plot_compara2(saida_li2, saida_nl, Tloop*iteracoes)

