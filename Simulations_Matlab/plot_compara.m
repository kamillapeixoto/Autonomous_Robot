function  plot_compara (ytot_dl, yOL, t_max, Q, R)

t_dl = linspace(0, t_max, length(ytot_dl));
t_OL = linspace(0, t_max, length(yOL));

%Comparar ode e SS CInematico
figure() %x, y
plot(yOL(:,1),yOL(:,2), 'b')
hold on
plot(ytot_dl(:,1),ytot_dl(:,2),'--r');
title('Trajetoria xy')
legend('Nao Linear', 'Linear')

figure() %x
plot(t_OL, yOL(:,1), 'b')
hold on
plot(t_dl, ytot_dl(:,1),'--r');
title('Trajetoria x')
legend('Nao Linear', 'Linear')


figure() % y
plot(t_OL, yOL(:,2), 'b')
hold on
plot(t_dl, ytot_dl(:,2),'--r');
title('Trajetoria y')
legend('Nao Linear', 'Linear')

figure() % wd
plot(t_OL, yOL(:,4),'b')
hold on
plot(t_dl, ytot_dl(:,4),'--r');
title('Velocidade direita')
legend('Nao Linear', 'Linear')

figure() %we
plot(t_OL, yOL(:,5),'b')
hold on
plot(t_dl, ytot_dl(:,5),'--r');
title('Velocidade esquerda')
legend('Nao Linear', 'Linear')

figure() % erro we
plot(t_OL, yOL(:,6),'b')
hold on
plot(t_dl, ytot_dl(:,6),'--r');
title('Erro velocidade esquerda')
legend('Nao Linear', 'Linear')

figure() %erro we
plot(t_OL, yOL(:,7),'b')
hold on
plot(t_dl, ytot_dl(:,7),'--r');
title('Erro Velocidade direita')
legend('Nao Linear', 'Linear')

figure() %theta
plot(t_OL, wrapTo360(rad2deg(yOL(:,3))),'b');  
hold on
plot(t_dl, wrapTo360(rad2deg(ytot_dl(:,3))),'--r');      % theta
title('Theta')
legend('Nao Linear', 'Linear')

end

