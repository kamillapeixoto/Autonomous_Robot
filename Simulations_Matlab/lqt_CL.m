function [K_lqt, F, modelo_cl] = lqt_CL(modelo_ol,Q_opt, R_opt)

global Tpid;

    % Calcula o ganho K (Feedback)
    K_lqt = dlqry(modelo_ol.A, modelo_ol.B,modelo_ol.C, modelo_ol.D, Q_opt, R_opt);
   % K_lqt = ones(2,7);
    
    %Fecha a malha
    modelo_cl   = modelo_ol;
    modelo_cl.A = modelo_ol.A - modelo_ol.B*K_lqt;
    
    %Calcula o ganho F (diretamente na referencia)
    F = (modelo_ol.C*(eye(size(modelo_cl.A))-modelo_cl.A)^(-1)*modelo_ol.B)^(-1);
    %F = inv(modelo_ol.C*inv(eye(size(modelo_cl.A))-modelo_cl.A)*modelo_ol.B)
    %F = ones(2);
 
end