function robo_cin = cinematico_SS(x0, y0, theta0, wd0, we0)

global mw R mc d L m Iw Ic I robo_dnm Ktorque Atrito Res Kp Ki vel_cl;
global Tpid;

%wd0 = 1e-6; % Pois a entrada é degrau
%we0 = 1e-6;

% Modelo cinemático (discreto):             https://pdfs.semanticscholar.org/dfe3/db26ec6f7b24060e0a6aa6648ce61eda703d.pdf
syms x y theta wd we;
    % define
    cin = (R/2)*Tpid*[cos(theta), cos(theta);
                      sin(theta), sin(theta);
                            1/L ,      -1/L]   * [wd; we] + [x;y;theta];
                        
    % Lineariza - Nao pode aplicar a jacobiana direto para as entradas por causa da regra
    % da cadeia, já que wd e we são funcões de theta
    cin_linX = simplify(jacobian(cin, [x y theta])); 
    cin_linU = simplify(jacobian(cin, [wd we]));                                                         
    
    % Aplica o ponto de operação na Jacobiana
    cin_linX0 = subs(cin_linX, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    cin_linU0 = subs(cin_linU, [x, y, theta, wd, we], [x0, y0, theta0, wd0, we0]);
    % Define em SSpo
    robo_cin = ss(double(cin_linX0),double(cin_linU0),eye(2,3),zeros(2,2), Tpid);
   

end

