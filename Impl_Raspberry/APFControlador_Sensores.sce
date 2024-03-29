
//          APF + CONTROLADOR
// Este programa define a posição do robô e do obstáculo e planeja a trajetoria,
// bem como as velocidades necessárias para que o robô siga a mesma.
// 12/12/2016


// Abre o diretorio
getd('/media/pi/5F25-1788/Versao2704/');


// Carrega os parametros
exec('/media/pi/5F25-1788/Versao2704/parametros_apf.sci', -1);

//Inicia a comunicação com o arduino
//arduino_com = openserial("COM11","115200,n,8,1") // para Windows
arduino_com = openserial("/dev/ttyACM1","115200,n,8,1") // para Linux "


// Verifica se a comunicaçao está ok
if arduino_com == -1 then
  disp("Erro em Comunicação Serial");
  else
  disp("Comunicação Serial OK");
end
                                                                                    
// Define a quantidade de characteres que serao recebidos por vez [x y theta]
qtde_character = 20; // Verificar quantos serão necessários.

// Define os valores da posição x,y inicial do robô e o angulo
// Note que futuramente isso será definido pelos sensores Bluetooth
robo_pos = [0,0];
theta_pos = %pi/4;

// Define os valores da posição do obstáculo
// Note que futuramente isso será definido pelos sensores de Berguem
obst_pos_fix = [200,200];

// Define os valores da posição do objetivo
final_pos = [2,2]; 
distancia = norm(robo_pos - final_pos);
                                                                                                                                                                                                                                                                
//Incia algumas variáveis
m =1;               // Qtd de mensagens recebidas
posicao1 = '';      // Lido
velocidade = [0 0];

lixo = readserial(arduino_com);

// writeserial(arduino_com, strcat(string([200 200]), ","));
while distancia > 0.1 then
//                                                                                        while arduino_com ~= -1 
    tic ();
  
//Início da comunicação serial
recebido = readserial(arduino_com, 1);
                                                                                        //disp(recebido)
    if ascii(recebido)== 2 then //Inicio do texto
            m = m+1; // Conta as mensagens recebidas
          
            // Continua lendo
            while ascii(recebido) ~= 3 // Final do texto
              recebido = readserial(arduino_com, 1);
              if ~isempty(recebido)
                posicao1  = msprintf('%s%s',posicao1, recebido)
              end
            end // while ascii(recebido) ~= 3           

            // Divide os dados recebidos e atribui valores de posicao
            dados(m,:) = strtod(strsplit(posicao1,','))';
            disp(dados)
            robo_pos(m,:) = dados(m,1:2);
            //plot(robo_pos(m,1), robo_pos(m,2))
            theta_pos(m) = dados(m,3);
            
            obst_pos =[];
            //obst_pos = [obst_pos_fix; obs_mov];
            // Calcula o próximo ponto utilizando APF
            forca_tot(m,:) = APF (robo_pos(m,:), final_pos, obst_pos_fix, a_max, etta, xi, ro_o, a_min);
        
            // Calcula as velocidades em cada roda a partir da força total
            velocidade(m, :) = controlador(forca_tot(m,:),theta_pos(m),kr,kl); // [u1 u2]
            
            vel_string(m) = strcat(string([velocidade(m,1) velocidade(m,2)]), ",");
            
            //Envia as velocidades desejadas para o Arduino
            writeserial(arduino_com, vel_string(m)); 
                  
            posicao1 = "";
            distancia = norm(robo_pos(m,:) - final_pos);
    end // if ascii(recebido)== 2 then
    
 t = toc();
//disp(t);    
 end // distancia > 0.01 then
      
     velocidade(m,:) = [0, 0];      
     vel_string(m) = strcat(string([velocidade(m,1)]), ",");
            
     //Envia as velocidades desejadas para o Arduino
     writeserial(arduino_com, vel_string(m)); 
     disp('Ok!')
     
          velocidade(m,:) = [0, 0];      
     vel_string(m) = strcat(string([velocidade(m,1)]), ",");
            
     //Envia as velocidades desejadas para o Arduino
     writeserial(arduino_com, vel_string(m)); 
     disp('Ok!')
     
          velocidade(m,:) = [0, 0];      
     vel_string(m) = strcat(string([velocidade(m,1)]), ",");
            
     //Envia as velocidades desejadas para o Arduino
     writeserial(arduino_com, vel_string(m)); 
     disp('Ok!')
     
          velocidade(m,:) = [0, 0];      
     vel_string(m) = strcat(string([velocidade(m,1)]), ",");
            
     //Envia as velocidades desejadas para o Arduino
     writeserial(arduino_com, vel_string(m)); 
     disp('Ok!')
          velocidade(m,:) = [0, 0];      
     vel_string(m) = strcat(string([velocidade(m,1)]), ",");
            
     //Envia as velocidades desejadas para o Arduino
     writeserial(arduino_com, vel_string(m)); 
     disp('Ok!')
            
closeserial(arduino_com)
