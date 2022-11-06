function [K_I, robo_I] = lqi_CL(robo, Q_opt, R_opt)

    global Tpid;

    % Define o novo SS com estado estendido
    [ma na] = size(robo.A);
    [mb nb] = size(robo.B);
    [mc1 nc] = size(robo.C);
    [md nd] = size(robo.D);
    
    
    robo_I_A = [robo.A zeros(7,2); -robo.C eye(2)];
    robo_I_B = [robo.B ; zeros(2)];
    robo_I_C = [robo.C zeros(2)];
    robo_I_D = zeros(2);
    
    robo_I = ss(robo_I_A, robo_I_B,robo_I_C,robo_I_D,Tpid);
    
    Q = robo_I_C*0.001*Q_opt*robo_I_C';
    
    %[K_I, S, E] = dlqry(robo_I_A, robo_I_B, robo_I_C,robo_I_D, Q, R_opt);
   %[K_I2, S, E] = dlqr(robo_I_A, robo_I_B, Q_opt, R_opt);
    [K_I, S, E] = dlqr(robo_I.A, robo_I.B, Q_opt, R_opt);

   %[K_I, S, E] = lqi(robo,Q_opt, R_opt);
 
    %Fonte: http://www.sbai2013.ufc.br/pdfs/4107.pdf
    robo_I.A = [robo.A-robo.B*K_I(:,1:7) -robo.B*K_I(:,8:9); -robo.C eye(nb)] ;
    robo_I.B = [zeros(ma,nb); eye(nb)];
    
%     
%     A_I = [(robo.A-robo.B*K_I(:,1:7)) robo.B*K_I(:,8:9); -robo.C zeros(mc1)];
%     B_I = [zeros(ma,nb); eye(nb)];
%     C_I = [-robo.C zeros(mc1,mc1)];
%     D_I =  zeros(md,nd);

    
  
end