%% PLOT APF

global etta xi ro_o a_max

parametros_APF();

pos_ini = [0.1 0.1];
pos_obj   = [2.5 2.5];
pos_obs   = [0.73,2.14;
            1.48,1.44;
             2.14,0.73];
              
tam_field = 0:0.2:3;    
a_max =1;

[x,y] = meshgrid(tam_field, tam_field);

tam = size(x);
Fx= zeros(tam);
Fy= zeros(tam);

qtd_elementos = tam(1)*tam(2);


for i=1:qtd_elementos
    
 F_apf = APF([x(i),y(i)], pos_obj, pos_obs);
 Fx(i) = F_apf(1);
 Fy(i) = F_apf(2);
 
end

figure()
q = quiver(x,y, Fx, Fy)
q.Color= 'black'
[FT_ang Ft_intensidade] = cart2pol(Fx, Fy);
% figure()
% pcolor(x,y,Ft_intensidade)
% colormap(hsv)




%%
%% Trajetoria

% dh=0.25;
% SX=-0.25:dh:6.5;
% SY=-0.25:dh:6.5;
% [SX,SY]=meshgrid(SX,SY);

xw = 345;
yw = 330;
%figure('position', [0, 0, xw, yw])
%surf(SX,SY,zeros(size(SX,2),size(SY,2)),'FaceColor','white', 'EdgeColor', [0.5,0.5,0.5]);
%view(2)
hold on

font_axis  = 10;
font_legen = 10;
font_label = 10;


xlabel('x (m)','FontSize',font_label)
ylabel('y (m)','FontSize',font_label)
set(gca,'fontsize',font_axis)
grid on
plot(pos_ini(:,1), pos_ini(:,2),'xb','MarkerSize',15, 'LineWidth',2);
plot(pos_obj(:,1), pos_obj(:,2),'xk','MarkerSize',18, 'LineWidth', 5);
hold on
plot(pos_obj(:,1), pos_obj(:,2),'xy','MarkerSize',15,'LineWidth',2);
hold on
plot(pos_obs(:,1), pos_obs(:,2), 'rx','MarkerSize',15, 'LineWidth',3);
hold on
% plot(setp(:,1), setp(:,2), '--','MarkerSize',8, 'LineWidth',2, 'Color',  [0.7,0.7,0.7]);
hold on
% 
% grid on
% radius = 1;
% ang=0:0.01:2*pi;
% xp=radius*cos(ang);
% yp=radius*sin(ang);
% h4 = plot(pos_obs(1,1)+xp,pos_obs(1,2)+yp, '-.k');
% h5 = plot(pos_obs(2,1)+xp,pos_obs(2,2)+yp, '-.k');
% h6 = plot(pos_obs(3,1)+xp,pos_obs(3,2)+yp, '-.k');



     xlim([0 3])
     ylim([0 3])