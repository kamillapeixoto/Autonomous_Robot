%% Malha fechada com LQT

contro    = []; % Verificar a controlabilidade do modelo OL
saida_li  = []; % Resultado da simulação linear
saida_nl  = []; % Resultado da simulação linear
saida_li2 = [];

%Estados iniciais
est     = 0.001*ones(1,7);
% est(1:2) = [0.1 0.1];
% est(3) = pi/4;
estados = est;

y = est;
y_lqt = est;

tempo = 0:Tpid:Tloop;
%tempo = [0 Tpid*10];

u1 = ones(1,length(tempo));
u2 = ones(1,length(tempo));
u = 1*[u1;u2];

Q_opt = 4001*eye(2);
R_opt = 1*eye(2);
%R_opt = [0.008 0;0 0.02];

y_out = est;
ref = 0;
rapto = 0.3;

% Inicia a simulação
while (max(abs(estados(end,1:2) - u(:,1)')) > 0.1)
  %for ref =1:500   
    
   % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(3),estados(4), estados(5));

      % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
             
 
    % Calcula os ganhos do LQT
    [K_lqt, F_lqt, modelo_cl] = lqt_CL(robo_sim,Q_opt,R_opt); 
        
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    [y_dl, t_dl, x_dl] = lsim(modelo_cl, F_lqt*u, tempo, estados);
    saida_li2 = [saida_li2; x_dl];
    
    
    
   for i =1:13
        u1 = F_lqt*u(:,1) - K_lqt*y_out(end,:)'; 
    
   [t_out, y_out] = ode45(@(t,y) robo_LQT(t,y, u1),[0 Tpid], estados);
   saida_nl = [saida_nl; y_out];
   estados = y_out(end,:);
   
  
   end
      % Perturbação
    if ref == 9
        y_out(end,1:2) = y_out(end,1:2) + rapto;
    end
    % Atualiza as condições iniciais
      estados = y_out(end,:);

      ref= ref+1
end

%%
temp = linspace(0, Tloop*ref, length(saida_nl));
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

figure()
plot(temp(1:5378),saida_nl(1:5378,1), 'k', 'LineWidth',3)
hold on
plot(temp(5378:5379),saida_nl(5378:5379,1), '--k', 'LineWidth',2)
plot(temp(5379:end),saida_nl(5379:end,1), 'k', 'LineWidth',3)
xlabel('Tempo (s)')
ylabel('x(m)')
legend('Percurso','Rapto')
grid on

figure()
plot(temp(1:5378),saida_nl(1:5378,2), 'k', 'LineWidth',3)
hold on
plot(temp(5378:5379),saida_nl(5378:5379,2), '--k', 'LineWidth',2)
plot(temp(5379:end),saida_nl(5379:end,2), 'k', 'LineWidth',3)
xlabel('Tempo (s)')
ylabel('y(m)')
legend('Percurso','Rapto')
grid on

subplot(1,2,2)
plot(temp, saida_nl(:,2), 'Color', [0.7 0.7 0.7],'LineWidth',3)
xlabel('Tempo (s)')
ylabel('y(m)')


figure()
plot(temp,wrapTo360(rad2deg(saida_nl(:,3))), 'k', 'LineWidth',3)
xlabel('Tempo(s)')
ylabel('\theta (º)')

legend('Velocidade direita','Velocidade esquerda', 'theta')

figure()
plot(saida_nl(1:5378,1), saida_nl(1:5378,2), 'k', 'LineWidth',3)
hold on
plot(saida_nl(5378:5379,1), saida_nl(5378:5379,2), '--k', 'LineWidth',2)
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
plot(saida_nl(5379:end,1), saida_nl(5379:end,2), 'k', 'LineWidth',3)
% xlim([0.75 1.05])
% ylim([0.75 1.05])
legend('Percurso','Rapto','Posição Objetivo','Círculo de Precisão')
xlim([-0.05 1.15])
ylim([-0.05 1.15])

figure()
plot(saida_nl(:,1), saida_nl(:,2), 'k', 'LineWidth',3, )
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
legend('Percurso','Posição Objetivo','Círculo de Precisão')
xlim([-0.05 1.15])
ylim([-0.05 1.15])


%%

plot_compara2(saida_li2, saida_nl, Tpid*ref)
