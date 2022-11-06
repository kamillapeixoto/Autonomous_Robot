%% Malha fechada com LQI

global tensao_tot tempo_tot precisao;

contro    = []; % Verificar a controlabilidade do modelo OL
saida_li  = []; % Resultado da simulação linear
saida_nl  = []; % Resultado da simulação linear
saida_li2 = [];

%Estados iniciais
est = 0.001*ones(1,9);
est(1:2) = [0.1 0.1];
est(3) = pi/4;
%est = 1*pi/4*ones(1,9);
estados = est;
y_lqi = est;
y_lqi2 = est';

est_li = est;

tempo = 0:Tpid:Tloop;
%tempo = [0 Tpid*10];

u1 = ones(1,length(tempo));
u2 = ones(1,length(tempo));
u = 1*[u1;u2];

Q_opt = 1*eye(9);
 Q_opt(8,8) = 1000;
 Q_opt(9,9) = Q_opt(9,9);
R_opt = 1*eye(2);

y_out = est;
ref = 0;
tensao_soma = 0;
erro_soma = 0;

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
             
 
    % Calcula os ganhos do LQT
    [K_lqi, modelo_cl] = lqi_CL(robo_sim,Q_opt,R_opt); 
        
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    [y_dl, t_dl, x_dl] = lsim(modelo_cl, u, tempo, est_li(end,:));
    saida_li = [saida_li; x_dl];
    
    est_li = x_dl;
    
    
    for i =1:14
       esf = -K_lqi*y_out(end,:)';
       
        tensao_tot = [];
        tempo_tot = [];
       erro_soma = erro_soma + sqrt(sum((u(:,1) - y_out(end,1:2)').^2));
       
       [t_out, y_out] = ode45(@(t,y) robo_LQI(t,y, u,esf),[0 Tpid], estados);
       
       
       saida_nl = [saida_nl; y_out];
       estados = y_out(end,:);
       tensao_soma = tensao_soma + ...
                      sum(sum(abs(tensao_tot(:,1:end-1)).*diff(tempo_tot)'));
        
    end
%     
%     for i =1:14
%         y_lqi2 = robo_LQI_dy(u(:,1), K_lqi)*Tpid+y_lqi2;
%         saida_li2 = [saida_li2; y_lqi2'];
%     end
%     
    % Atualiza as condições iniciais
 
    estados = y_out(end,:);

      ref= ref+1
end


temp = linspace(0, Tloop*ref, length(saida_nl));
figure()
plot(temp,saida_nl(:,4), 'k', 'LineWidth',2)
hold on
plot(temp, saida_nl(:,5), 'Color', [0.7 0.7 0.7],'LineWidth',2)
xlabel('Tempo (s)')
ylabel('Velocidade (rad/s)')
legend('Direita', 'Esquerda')


plot(temp,wrapTo360(rad2deg(saida_nl(:,3)))/10)
legend('Velocidade direita','Velocidade esquerda', 'theta')

figure()
plot(saida_nl(:,1), saida_nl(:,2), 'k', 'LineWidth',2)
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

xlim([-0.05 1.15])
ylim([-0.05 1.15])


%plot_compara2(saida_li, saida_nl, tempo(end)*ref)
