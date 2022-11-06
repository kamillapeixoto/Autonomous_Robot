function plot_ajuste(iteracao, esforco_pid, erro, esforco_traj, pesos)


    % Define a fonte padrao
    set(0,'DefaultTextFontname', 'Arial')
    set(0,'DefaultTextFontSize', 10)
    set(0,'DefaultAxesFontName', 'Arial')
    set(0,'DefaultAxesFontSize', 10)
    
    % Define tamanho das imagens
    xw = 650;
    yw = 280;
    
    % Plota iteracoes
    figure('position', [50, 50, xw, yw])
    plot(pesos,iteracao','-ok','MarkerSize',4, 'MarkerFaceColor', [0 0 0])
    ylim([0 260])
    xlim([(pesos(1)-1),  (pesos(end)+1)])
    ylabel('Iterações')
    xlabel('Peso Matriz Q')
    grid on

    % Plota fsforço de controle do PID
    figure('position', [50, 50, xw, yw])
    plot(pesos,esforco_pid,'-ok','MarkerSize',4, 'MarkerFaceColor', [0 0 0])
    xlim([(pesos(1)-1),  (pesos(end)+1)])
    ylabel('Esforço de Controle do PID')
    xlabel('Peso Matriz Q')
    grid on

    % Plota erro
    figure('position', [50, 50, xw, yw])
    plot(pesos,erro,'-ok','MarkerSize',4, 'MarkerFaceColor', [0 0 0])
    xlim([(pesos(1)-1),  (pesos(end)+1)])
    ylabel('Erro (m)')
    xlabel('Peso Matriz Q')
    grid on

    
    % Plota fsforço de controle do controlador de trajetória
    figure('position', [50, 50, xw, yw])
    plot(pesos,esforco_traj,'-ok','MarkerSize',4, 'MarkerFaceColor', [0 0 0])
    xlim([(pesos(1)-1),  (pesos(end)+1)])
    ylabel('Esforço de Controle do LQT')
    xlabel('Peso Matriz Q')
    grid on

end

