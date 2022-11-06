function robo = robo_modelo_ss(x0, y0, theta0, wd0, we0)

global mw R mc d L m Iw Ic I robo_dnm Ktorque Atrito Res Kp Ki vel_cl;
global Tpid;

% Estados
% 1 - x
% 2 - y
% 3- theta
% 4 - wd
% 5 - we
% 6 - erro  - erro de velocidade do motor direito
% 7 - erro  - erro de velocidade do motor esquerdo


% Modelo do Robô

% PI
PI_r = tf([Kp Ki],[1 0]);
PI_l = tf([Kp Ki],[1 0]);

% Transforma para SS
[Ar, Br, Cr, Dr] = tf2ss(PI_r.num{1}, PI_r.den{1});
PI_rss = ss(Ar, Br, Cr, Dr);

[Ar, Br, Cr, Dr] = tf2ss(PI_l.num{1}, PI_l.den{1});
PI_lss = ss(Ar, Br, Cr, Dr);

%Torque
tau1r = (Ktorque/Res)*PI_rss;
tau1l = (Ktorque/Res)*PI_lss;

tau2r    = tf((Ktorque^2/Res + Atrito),1);
tau2l    = tf((Ktorque^2/Res + Atrito),1);

robo_dnm.OutputName = {'wd', 'we'};
robo_dnm.InputName  = {'torqd', 'torqe'};

tau1r.OutputName    = 'torqd1';
tau1l.OutputName    = 'torqe1';

tau1r.InputName    = 'ed';
tau1l.InputName    = 'ee';

tau2r.InputName     = 'wd';
tau2l.InputName     = 'we';

tau2r.OutputName    = 'torqd2';
tau2l.OutputName    = 'torqe2';

somad = sumblk('ed = rd - wd');
somae = sumblk('ee = re - we');


somatd = sumblk('torqd = torqd1 - torqd2');
somate = sumblk('torqe = torqe1 - torqe2');

vel = connect(robo_dnm, tau2r, tau2l,somad,somae,tau1r, tau1l,somatd, somate,  {'rd', 're'},{'wd','we'});

% Discretiza
vel_cl_d = c2d(vel, Tpid);


% % Modelo cinemático (discreto):             https://pdfs.semanticscholar.org/dfe3/db26ec6f7b24060e0a6aa6648ce61eda703d.pdf
% syms x y theta wd we;
%     % define
%     cin = (R/2)*Tpid*[cos(theta), cos(theta);
%                       sin(theta), sin(theta);
%                             1/L ,      -1/L]   * [wd; we] + [x;y;theta];
%     % Lineariza - Nao pode aplicar a jacobiana direto para as entradas por causa da regra
%     % da cadeia, já que wd e we são funcões de theta
%     cin_linX = simplify(jacobian(cin, [x y theta])); 
%     cin_linU = simplify((R/2)*Tpid*[(cos(theta)-(wd+we)*sin(theta)/(4*L)), (cos(theta)+(wd+we)*sin(theta)/(4*L));... %4L, pois vk = (wd + we)/2
%                          (sin(theta)+(wd+we)*cos(theta)/(4*L)), sin(theta)+(wd+we)*cos(theta)/(4*L);...
%                                     1/L                    , -1/L]);
%                                                             
%     % Aplica o ponto de operação na Jacobiana
%     cin_linX0 = subs(cin_linX, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
%     cin_linU0 = subs(cin_linU, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
%     % Define em SSpo
%     robo_cin = ss(double(cin_linX0),double(cin_linU0),eye(2,3),zeros(2,2), Tpid);
%      
%     % Calcula o novo modelo completo
%     robo = robo_cin * vel_cl_d;



% Modelo cinemático (discreto):             https://pdfs.semanticscholar.org/dfe3/db26ec6f7b24060e0a6aa6648ce61eda703d.pdf
syms x y theta wd we;
    % define
    cin = (R/2)*Tpid*[cos(theta), cos(theta);
                      sin(theta), sin(theta);
                            1/L ,      -1/L]   * [wd; we] + [x;y;theta];
                        
    % Lineariza - Nao pode aplicar a jacobiana direto para as entradas por causa da regra
    % da cadeia, já que wd e we são funcões de theta
    cin_linX = simplify(jacobian(cin, [x y theta])); 
   % cin_linU = simplify((R/2)*Tpid*[(cos(theta)-(wd+we)*sin(theta)/(4*L)), (cos(theta)+(wd+we)*sin(theta)/(4*L));... %4L, pois vk = (wd + we)/2
   %                    (sin(theta)+(wd+we)*cos(theta)/(4*L)), (sin(theta)-(wd+we)*cos(theta)/(4*L));...
   %                               1/L                    , -1/L]);
                                                            
     cin_linU = simplify(jacobian(cin, [wd we]));
     

    % Aplica o ponto de operação na Jacobiana
    cin_linX0 = subs(cin_linX, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    cin_linU0 = subs(cin_linU, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    % Define em SSpo
    robo_cin = ss(double(cin_linX0),double(cin_linU0),eye(2,3),zeros(2,2), Tpid);
   
    robo = robo_cin * vel_cl_d;


end

