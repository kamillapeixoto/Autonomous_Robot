%% Código de teste
% Esse código testa os modelos com diferentes representações


global mw R mc d L m Iw Ic I robo_dnm Ktorque Atrito Res Kp Ki sat vel_min;
global Tpid Tloop Tsim Nloop;
global K_I y_lqi y_lqi2 y_lqt zm;
global Fs Fk atr_s atr_k precisao;

parametros_robo();
parametros_simulacao();

tempo = 0:Tpid:Tloop*10;
entrada = [5*ones(size(tempo)); 3*ones(size(tempo))];
    
%% Testar controle de velocidade
[tvel,yvel] = ode45(@(t,y) velocidade_CL(t,y,entrada), tempo,zeros(4,1));
figure();
plot(tvel, yvel);

%% Testar controle de velocidade SS
[yve    lss,tvelss] = lsim(velocidade_CL_SS,entrada,tempo);
figure();
plot(tvelss, yvelss);

% Comparar ode e ss
figure()
plot(tvel, yvel(:,1), 'b');
hold on
plot(tvel, yvel(:,2),'r');
plot(tvelss, yvelss(:,1), '--b');
plot(tvelss, yvelss(:,2), '--r');

% Conclusoes: 
% A saida de velocidade em SS só possui erro nulo se adicionar mais um polo na origem.
% Os dois modelos de velocidade estao identicos

%% Testar Modelo cinemático
cin_tempo = 0:Tpid:Tloop;
[tcin,ycin] = ode45(@(t,y) cinematico(t,y, entrada(1), entrada(2)), cin_tempo, 0*ones(3,1));
% figure();
% plot(ycin(:,1),ycin(:,2));
% figure()
% plot(wrapTo360(rad2deg(ycin(:,3))));  

%% Testar Modelo cinemático SS 

xtotcin = [];
contro = [];

pos0 = 0*ones(3,1);

x_pos = pos0(1);
y_pos = pos0(2);
t_pos = pos0(3);

vel_d = 0;
vel_e = 0;

vel_d = entrada(1,1);
vel_e = entrada(2,1);


% deltaud = entrada(1,1)-vel_d;
% deltaue = entrada(2,1)-vel_e;

deltax = zeros(1,3);
    
    % Calcula o novo modelo 
    robocin = cinematico_SS(pos0(1), pos0(2),pos0(3), vel_d,vel_e);
    
    ctr = length(robocin.A) - rank(ctrb(robocin)); % = 0 controlavel
    contro = [contro ~ctr];  
    
    
% Inicia a simulação
for i = 1:(length(cin_tempo)-1)

     
    %Aplica a entrada deltau=0, pois a entrada é degrau, com as condições
    %iniciais iguais a zero tambem, já que no inicio da simulação, o estado
    %coincide com o ponto de linearização, logo deltax = 0.
    [y_outcin, tcin, x_outcin] = lsim(robocin, [vel_d*ones(2,1) vel_e*ones(2,1)], [0 Tpid], deltax(end,:));
    
    deltax = x_outcin(end,:);
    
   % Atualiza a posição atual
%     x_pos = [x_pos; x_outcin(:,1)+fx(1)];
%     y_pos = [y_pos; x_outcin(:,2)+fx(2)];
%     t_pos = [t_pos; x_outcin(:,3)+fx(3)];
    x_pos = [x_pos; deltax(end,1)+x_pos(end)];
    y_pos = [y_pos; deltax(end,2)+y_pos(end)];
    t_pos = [t_pos; deltax(end,3)+t_pos(end)];
        
    
%vel_d = vel_d+deltau;
    %vel_e = vel_e+deltau;
end
% figure()
% plot(xtotcin(:,1),xtotcin(:,2))
% figure()
% plot(wrapTo360(rad2deg(xtotcin(:,3))));      % theta

%Comparar ode e SS CInematico
figure()
plot(ycin(:,1),ycin(:,2),'b');
hold on
plot(x_pos,y_pos, '--r')

figure()
plot(wrapTo360(rad2deg(ycin(:,3))),'b');  
hold on
plot(wrapTo360(rad2deg(t_pos)),'--r');      % theta


%% Testar modelo continuo e não linear em malha aberta
tempo_OL = 0:Tpid:Tloop*20;
entrada = [linspace(0,5,10);linspace(0,5,10)];
yOL = []
xOL =zeros(7,1);
for i = 1:10
    [tOL,yOL2, xOL] = ode45(@(t,y2) robo_modelo_OL(t,y2,entrada(:,i)), tempo_OL, xOL);
    yOL =[yOL; yOL2];
    xOL = yOL2(end,:);
end
figure();
plot(yOL(:,1),yOL(:,2));                     % x y
figure(); 
plot(tOL,wrapTo360(rad2deg(yOL(:,7))));      % theta

%% Testar modelo continuo e não linear em malha aberta com SATURAÇAO E ZONA MORTA
tempo_OL = 0:Tpid:Tloop*20;
yOL = []
xOL =zeros(7,1);
entrada = [linspace(3,5,5);linspace(0,5,5)];
for i = 1:5
    [tOL,yOL2, xOL] = ode45(@(t,y2) robo_modelo_OL_NL(t,y2,entrada(:,i)), tempo_OL, xOL);
    yOL =[yOL; yOL2];
    xOL = yOL2(end,:);
end
figure();
plot(yOL(:,4));                     % x y
figure(); 
plot(yOL(:,5));   


%% Testar modelo discreto e linear em malha aberta SS
% Conclusao: Todos modelos são instáveis, porém controláveis

estabi = []; % Verificar a estabilidade do modelo OL
contro = []; % Verificar a controlabilidade do modelo OL
ytot_dl  = []; % Resultado da simulação
deltax = zeros(7,1);

vel_d = 5;
vel_e = 3;

est =0*ones(3,1);

% Calcula o novo modelo completo
    robo = robo_modelo_ss(est(1),est(2),est(3), vel_d, vel_e);
 % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
    
    % Verifica a estabilidade 
    estabi = [estabi isstable(robo_sim)];
    
    
% Inicia a simulação
%for i = 1:(length(cin_tempo)-1)
                                                      
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    [y_dl, t_dl, x_dl] = lsim(robo_sim,[vel_d*ones(length(tempo_OL),1) vel_e*ones(length(tempo_OL),1)], tempo_OL, deltax);
    ytot_dl = [ytot_dl; x_dl];
    
    % Atualiza as condições iniciais
     deltax = x_dl(end,:);
      
%end

% %Plotar trajetorias
% figure()
% plot(ytot_dl(:,1),ytot_dl(:,2));
% figure()
% plot(wrapTo360(rad2deg(ytot_dl(:,3))));



%Comparar ode e SS CInematico
figure() %x, y
plot(yOL(:,5),yOL(:,6), 'b')
hold on
plot(ytot_dl(:,1),ytot_dl(:,2),'--r');

figure() % wd
plot(yOL(:,1),'b')
hold on
plot(ytot_dl(:,4),'--r');

figure() %we
plot(yOL(:,2),'b')
hold on
plot(ytot_dl(:,5),'--r');

figure() %we
plot(yOL(:,3),'b')
hold on
plot(ytot_dl(:,6),'--r');


figure() %we
plot(yOL(:,4),'b')
hold on
plot(ytot_dl(:,7),'--r');

figure() %theta
plot(wrapTo360(rad2deg(yOL(:,7))),'b');  
hold on
plot(wrapTo360(rad2deg(ytot_dl(:,3))),'--r');      % theta

%% Sistema em malha fechada com LQI

%% Malha fechada com LQI SS

estabi = []; % Verificar a estabilidade do modelo OL
contro = []; % Verificar a controlabilidade do modelo OL
ytot_dl  = []; % Resultado da simulação
ytot2 =[];

est = 0.1*ones(9,1);
estados = est;
%tempo = 0:Tpid:30;
tempo = 0:Tpid:100*Tpid;
u1 = ones(1,length(tempo));
u2 = ones(1,length(tempo));
u = [u1;u2];

    
   % Inicia a simulação
for ref = 1:(length(cin_tempo)) 
    
    % Calcula o novo modelo completo
    robo = robo_LQI_ss(est(1),est(2),est(3),5,3);
      % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
    
    % Verifica a estabilidade 
    estabi = [estabi isstable(robo_sim)];

                                                                
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    [y_dl, t_dl, x_dl] = lsim(robo_sim, u, tempo, est);
    ytot_dl = [ytot_dl; x_dl];
    
    
   %Aplica a entrada u com as condições iniciais iguais à ultima simulação
   KI_nl = [K_I(:,4) K_I(:,5) K_I(:,6) K_I(:,7) K_I(:,1) K_I(:,2) K_I(:,3) K_I(:,8) K_I(:,9)];
   [t_out, y_out] = ode45(@(t,y) robo_LQI(t,y, u, KI_nl),tempo, estados);
   ytot2 = [ytot2; y_out];
    
    % Atualiza as condições iniciais
    est = x_dl(end,:);
    estados = y_out(end,:);
      
end
%{
for j = 1:9
    figure()
    plot(ytot_dl(:,j))
    hold on
    plot(ytot2(:,j));
end
  %}  

%Comparar ode e SS CInematico
figure() %x, y
plot(ytot2(:,5),ytot2(:,6), 'b')
hold on
plot(ytot_dl(:,1),ytot_dl(:,2),'--r');

figure() % wd
plot(ytot2(:,1),'b')
hold on
plot(ytot_dl(:,4),'--r');

figure() %we
plot(ytot2(:,2),'b')
hold on
plot(ytot_dl(:,5),'--r');

figure() %we
plot(ytot2(:,3),'b')
hold on
plot(ytot_dl(:,6),'--r');


figure() %we
plot(ytot2(:,4),'b')
hold on
plot(ytot_dl(:,7),'--r');

figure() %theta
plot(wrapTo360(rad2deg(ytot2(:,7))),'b');  
hold on
plot(wrapTo360(rad2deg(ytot_dl(:,3))),'--r');      % theta

%%
figure()
plot(ytot_dl(:,1));
hold on
plot(ytot2(:,5));

figure()
plot(ytot_dl(:,6));
hold on
plot(ytot2(:,2));

%Plotar trajetorias
figure()
plot(ytot2(:,5));
hold on
plot(ytot2(:,6));

figure()
plot(wrapTo360(rad2deg(ytot2(:,7))));


%% Testar LQI com sistema nao linear e contínuo

ytot2  = [];
yplot  = [];
estabi = [];
contro = [];

estados = 0*ones(9,1);

%estado(7) = pi/4;

tempo = 0:Tpid:10;
u1 = ones(1,length(tempo));
u2 = ones(1,length(tempo));
u = [u1;u2];



% Define os pesos da função custo
Q_opt = 1*eye(9);
Q_opt(1:2,:) = 1*Q_opt(1:2,:);
R_opt = 1*eye(2);


        % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(3),5,3);
               
    % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
       %Calcula os ganhos
       [K_I, S, E] = lqi(robo_sim, Q_opt, R_opt);
       %robo_LQI_ss(estados(1),estados(2),estados(3));
    end
    contro = [contro ctr];
       
   %Aplica a entrada u com as condições iniciais iguais à ultima simulação
   [t_out, y_out] = ode45(@(t,y) robo_LQI(t,y, [ref ref], K_I), tempo, estados);
   ytot2 = [ytot2; y_out];

   
    yplot = [yplot;ref*u'];


%Plotar trajetorias
figure()
plot(ytot2(:,5));
hold on
plot(yplot(:,1)/100);
plot(ytot2(:,6));

figure()
plot(wrapTo360(rad2deg(ytot2(:,7))));



%% Testar LQI com sistema em espaço de estados. 
tempo = 0:Tpid:Tloop;
est = zeros(9,1);
robo = robo_modelo_ss(est(1),est(2),est(5),est(6),est(7));

u1 = 5*ones(1,length(tempo));
u2 = 10*ones(1,length(tempo));
u = [u1;u2];


ytot     = []; % Resultado da simulação
yplot    = []; % Referencia
estabi = []; % Verificar a estabilidade do modelo OL
contro = []; % Verificar a controlabilidade do modelo OL
ytotOD   = []; % Resultado da simulação

% LQI

% Define o novo SS com estado estendido
[ma na] = size(robo.A);
[mb nb] = size(robo.B);
[mc1 nc] = size(robo.C);
[md nd] = size(robo.D);

A_I = [robo.A zeros(nc, mc1); -robo.C eye(mc1)];
B_I = [robo.B; zeros(mc1, nb)];
C_I = [robo.C zeros(mc1,mc1)];
D_I = [robo.D];

robo_I = ss(A_I, B_I, C_I, D_I, Tpid);

l = length(A_I);
est = ones(l,1);                                                                                                                                                  

% Define os pesos da função custo
Q_opt = 1*eye(size(A_I));
aux = size(B_I);
R_opt = 1*eye(aux(2));


% Inicia a simulação
for ref = 1:1:Nloop
    
    % Calcula o novo modelo completo
    robo = robo_modelo_ss(est(1),est(2),est(3),est(4),est(5));
    
    % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
    
    % Verifica a estabilidade 
    estabi = [estabi isstable(robo_sim)];
    
                                                          
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    [y_outOD] = lsim(robo_sim, u, tempo, est);
    ytotOD = [ytotOD; x_out];
    
    % Atualiza as condições iniciais
    est = x_out(end,:);
      
end

%Plotar trajetorias
figure()
plot(ytot(:,1),ytot(:,2));
figure()
plot(wrapTo360(rad2deg(ytot(:,3))));



%%
estabi   = []; % Verificar a estabilidade do modelo OL
contro   = []; % Verificar a controlabilidade do modelo OL
estabi_I = []; % Verificar a estabilidade do modelo 
contro_I = []; % Verificar a controlabilidade do modelo
ytotLQI     = []; % Resultado da simulação
yplotLQI    = []; % Referencia
y_outLQI    = [];
ytotLQIss   = [];
tLQI = 0;
ytot = [];

% Define os pesos da função custo
Q_opt = 1*eye(9);
R_opt = 1*eye(2);

estados = 0.01*ones(9,1);
ref = 1;

%Define a entrada
tempo    = 0:Tpid:2;
enttraj  = ones(2,length(tempo));

% Inicia a simulação
for ref = 1:1:Nloop
    
    % Calcula o novo modelo completo
    robo = robo_modelo_ss(estados(1),estados(2),estados(5),estados(6),estados(7));
    
    % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ~ctr];
   
    %Calcula os ganhos
    [K_I, S, E] = lqi(robo_sim, Q_opt, R_opt);

     %Define a porção da entrada
     reftraj = enttraj*ref;
     
    %Aplica a entrada u com as condições iniciais iguais à ultima simulação
    %Modelo Linear
   % [y_outLQIss, tLQIss, x_outss] = lsim(robo_sim, reftraj, tempo, estados);
   % ytotLQIss = [ytotLQIss; x_outss];
   
    %Modelo Nao Linear
    t1 = (tLQI(end):Tpid:(tLQI*Tpid));
    [tLQI, y_outLQI] = ode45(@(t,y) robo_LQI(t,y, reftraj, K_I),[0 15], estados);
    ytot = [ytot; y_outLQI];
    
    % Atualiza as condições iniciais
    estados = y_outLQI(end,:);
      
end

%Plotar trajetorias
figure()
plot(ytot(:,5),ytot(:,6));
figure()
plot(wrapTo360(rad2deg(ytot(:,3))));


%%

tempo = 0:Tpid:Tloop;

% Inicia a simulação
for i = 0:1:Nloop
           
    % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ctr];
    % Verifica a estabilidade 
    estabi = [estabi isstable(robo_sim)];
                          
   %LQI 
   A_I = [robo_sim.A zeros(nc, mc); -robo_sim.C eye(mc)];
   B_I = [robo_sim.B; zeros(mc, nb)];
   C_I = [robo_sim.C zeros(mc,mc)];
   D_I = [robo_sim.D];
   
   
   robo_I.A = A_I;
   robo_I.B = B_I;
   robo_I.C = C_I;
   robo_I.D = D_I;
   
   % Verifica a controlabilidade
    ctr_I = length(robo_I.A) - rank(ctrb(robo_I)); % = 0 controlavel
    
    if(~ctr_I)              % So utiliza o modelo novo se ele for controlável
        robo_Isim = robo_I;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro_I = [contro_I ctr_I];
       
   %Calcula os ganhos
   [K_I, S, E] = dlqr(robo_I.A, robo_I.B, Q, R);

   %Fecha o loop
   robo_Icl = robo_I;
   
   Acl = (robo_I.A -robo_I.B*K_I);   
   robo_Icl.A = Acl;
   robo_Icl.B = [zeros((l-2),2);eye(2)];
   
   % Verifica a estabilidade 
   est = isstable(robo_Icl);
   if (est)
       robo_Icl2 = robo_Icl;
   end
    estabi_I = [estabi_I est];
       
    
   %Aplica a entrada u com as condições iniciais iguais à ultima simulação
   [y_out, t, x_out] = lsim(robo_Icl2, ref*u, tempo, estados);
   ytot = [ytot; x_out(:,1:3)];
    
   % Atualiza as condições iniciais
    estados = x_out(end,:);
    x0     = estados(end,1);
    y0     = estados(end,2);
    theta0 = estados(end,3);
    
    
    % Aplica o ponto atual na Jacobiana
    cin_linX0 = subs(cin_linX, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    cin_linU0 = subs(cin_linU, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    
    % Redefine o SS
    robo_cin.A = double(cin_linX0);
    robo_cin.B = double(cin_linU0);
    
    % Calcula o novo modelo completo
    robo = robo_cin * vel_cl_d;
      
    yplot = [yplot ref*u];
     ref = ref+1;
end

% %Plotar trajetorias
% figure()
% plot(ytot(:,1),ytot(:,2));
% figure()
% plot(ytot(:,3));
% %Plotar trajetorias

figure()
plot(ytot(:,1), ytot(:,2));
hold on
plot(yplot(1,:),yplot(2,:), 'r');


[xp,yp] = pol2cart(ytot(:,3), 0.1)
quiver(ytot(:,1), ytot(:,2), xp, yp)

tplot = 0:1:length(ytot)-1;
tplot = tplot*Tpid;
figure()
plot(tplot, ytot(:,2));
hold on
plot(tplot, yplot(2,:));
title('Posicao Y')
xlabel ('Tempo(s)')

figure()
plot(tplot, ytot(:,1));
hold on
plot(tplot, yplot(1,:));
title('Posicao X')
xlabel ('Tempo(s)')

figure()
plot(tplot, ytot(:,3));
title('Theta')
xlabel ('Tempo(s)')
annotation('arrow', [0.10 0.15],[0.10 0.15])


% %Plotar trajetorias
% figure()
% plot(y(:,1),y(:,2));
% figure()
% plot(y(:,3));

%% Lemniscata

t1 =linspace(0,3*pi,20000)';
tempo = 0:Tpid:(length(t1)*Tpid)';
x1=3*sin(t1)./(1+cos(t1).^2);
y1=4*sin(t1).*cos(t1)./(1+cos(t1).^2);
xa=[x1;x1(end)];
ya=[y1;y1(end)];

u = [xa ya];

uloop = floor(length(u)/(Nloop+1));

% Inicia a simulação
for ref = 1:1:Nloop
           
    % Verifica a controlabilidade
    ctr = length(robo.A) - rank(ctrb(robo)); % = 0 controlavel
    
    if(~ctr)              % So utiliza o modelo novo se ele for controlável
        robo_sim   = robo;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro = [contro ctr];
    % Verifica a estabilidade 
    estabi = [estabi isstable(robo_sim)];
    
        
                          
   %LQI 
   A_I = [robo_sim.A zeros(nc, mc1); -robo_sim.C eye(mc1)];
   B_I = [robo_sim.B; zeros(mc1, nb)];
   C_I = [robo_sim.C zeros(mc1,mc1)];
   D_I = [robo_sim.D];
   
   
   robo_I.A = A_I;
   robo_I.B = B_I;
   robo_I.C = C_I;
   robo_I.D = D_I;
   
   
   % Verifica a controlabilidade
    ctr_I = length(robo_I.A) - rank(ctrb(robo_I)); % = 0 controlavel
    
    if(~ctr_I)              % So utiliza o modelo novo se ele for controlável
        robo_Isim = robo_I;  % Caso não seja controlável, utiliza o modelo obtido no ponto anterior
    end
    contro_I = [contro_I ctr_I];
    
    
    
         
       
   %Calcula os ganhos
   [K_I, S, E] = lqi(robo, Q_opt, R_opt);

   %Fecha o loop
   robo_Icl = robo_I;
   
   Acl = (robo_I.A -robo_I.B*K_I);   
   robo_Icl.A = Acl;
   robo_Icl.B = [zeros((l-2),2);eye(2)];
   
   %
   % Verifica a estabilidade 
   est = isstable(robo_Icl);
   if (est)
       robo_Icl2 = robo_Icl;
   end
    estabi_I = [estabi_I est];
    
     
    
   %Define a porção da entrada
   index = round(((ref-1)*uloop+ref):1:(ref*uloop+ref));
    
   %Aplica a entrada u com as condições iniciais iguais à ultima simulação
   [t_out, y_out, x_out] = ode45(@(t,y) robo_LQI(t,y, entrada, K_I), [0 15], estados);
   ytot = [ytot; x_out(:,1:3)];
    
   
   % Atualiza as condições iniciais
    estados = x_out(end,:);
    x0     = estados(end,1);
    y0     = estados(end,2);
    theta0 = estados(end,3);
    
     
    
    % Aplica o ponto atual na Jacobiana
    cin_linX0 = subs(cin_linX, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    cin_linU0 = subs(cin_linU, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    
    % Redefine o SS
    robo_cin.A = double(cin_linX0);
    robo_cin.B = double(cin_linU0);
    
    % Calcula o novo modelo completo
    robo = robo_cin * vel_cl_d;
      
   yplot = [yplot u(index,:)'];
end

% %Plotar trajetorias
% figure()
% plot(ytot(:,1),ytot(:,2));
% figure()
% plot(ytot(:,3));
% %Plotar trajetorias

figure()
plot(ytot(:,1), ytot(:,2));
hold on
plot(yplot(1,:),yplot(2,:), 'r');


[xp,yp] = pol2cart(ytot(:,3), 0.1);
%quiver(ytot(:,1), ytot(:,2), xp, yp)

tplot = 0:1:length(ytot)-1;
tplot = tplot*Tpid;
figure()
plot(tplot, ytot(:,2),'o');
hold on
plot(tplot,yplot(2,:),'o');
title('Posicao Y')
xlabel ('Tempo(s)')

figure()
plot(tplot, ytot(:,1),'o');
hold on
plot(tplot, yplot(1,:),'o');
title('Posicao X')
xlabel ('Tempo(s)')

figure()
plot(tplot, ytot(:,3),'o');
title('Theta')
xlabel ('Tempo(s)')
