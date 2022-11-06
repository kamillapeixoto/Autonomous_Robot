# Autonomous_Robot
Electrical Engineering Final Project

## Table of contents
* [Project Overview](#project-overview)
* [Documentation](#documentation)
* [Simulations](#simulations)
* [Implementation](#implementation)
* [Setup](#setup)

## Project Overview

[English]
This project presents an autonomous nonholonomic differential drive mobile robot through processing entirelyembedded. For this purpose, an Artiﬁcial Potential Field (APF) algorithm was used for local path planning and trajectory controller. The proposed APF algorithm uses vortex ﬁeld to avoid null resultant potential ﬁeld points and predicts futurepositions using odometry as an alternative to compensate processing times and serial communication lags. Furthermore,the robot has an isolated and decentralized processing architecture using Arduino Mega and Raspberry PI as embedded systems. In addition, simulations were performed in MATLAB using a Quadratic Linear Tracker for trajectory control. Simulated and real experiments strongly suggest the effectiveness of the proposed path planning and control algorithms for real-time applications.

[Português]
Este trabalho apresenta um robô autônomo não holonômico de tração diferencial com processamento inteiramente embarcado e distribuído. Para isso, foi desenvolvido um planejamento local de trajetória baseado em Campos Potenciais Artificiais, cujo algoritmo inclui a Força de Vórtex para evitar pontos de campos potenciais resultantes nulos, e previsão de posições futuras como uma alternativa para compensar atrasos de processamento. O robô ainda conta com PIs e encoders para fazer o controle de velocidade das rodas em malha fechada e é capaz de se localizar no ambiente utilizando um método de odometria. As rotinas dos sensores de velocidade, a odometria e os controladores de velocidade são embarcados numa placa Arduino, enquanto o planejador de trajetória em um Raspbery PI utilizando software Scilab. A comunicação entre as placas é feita de forma serial. Além disso, foram realizadas simulações no software MATLAB utilizando um Rastreador Linear Quadrático para controle de trajetória para o sistema proposto. O resultados dos experimentos simulados e reais fortemente sugerem  a eficácia da metodologia proposta. 



## Documentation


[APPLICATION OF PATH PLANNING BASED ON ARTIFICIALPOTENTIAL FIELDS TO MOBILE ROBOTS WITH DISTRIBUTED PROCESSING](https://www.researchgate.net/publication/326431781_Application_of_path_planning_based_on_Artificial_Potential_Fields_to_mobile_robots_with_distributed_processing)

[MONTAGEM E CONTROLE DE TRAJETÓRIA DE  ROBÔS AUTÔNOMOS NÃO HOLONÔMICOS](https://www.overleaf.com/read/vtmyncphdbms)

Also:

[Comparative Evaluation of Fuzzy Maneuvering Controllers Robustness for Differential Drive Mobile Robots](https://proceedings.science/sbai-2019/trabalhos/comparative-evaluation-of-fuzzy-maneuvering-controllers-robustness-for-different)

[Fuzzy Maneuvering Controller applied to a Dynamic Model of a Differential Drive Mobile Robot](https://ieeexplore.ieee.org/document/8491684/)

[Fusão de sensores aplicada à localização de robôs móveis em ambientes fechados](https://www.researchgate.net/publication/326434120_Fusao_de_sensores_aplicada_a_localizacao_de_robos_moveis_em_ambientes_fechados)


## Simulations
**Software: MATLAB,
Hardware: Computer**



1.Models

[Test different model representations](Simulations_Matlab/main_verificacao.m)


2. Robot with LQT

[Find the "optimal" values to Q and R](Simulations_Matlab/main_lqt_ajuste.m)

[Simulation](Simulations_Matlab/main_lqt.m)

[Simulation with perturbation](Simulations_Matlab/main_lqt_perturb.m)


3. Robot with LQI

[Find the "optimal" values to Q and R](Simulations_Matlab/main_lqi_ajuste.m)

[Simulation](Simulations_Matlab/main_lqi.m)


4. APF

[Generate APF mapping](Simulations_Matlab/main_plot_APF.m)

[Robot with LQT and APF](Simulations_Matlab/main_lqtapf.m)



## Implementation

**Software: Scilab, 
Hardware: Raspberry Pi**

The APF and the path controller, both implemented on Scilab, compute, respectively, the force and wheel speed required to achieve the next position.Those speed values are transmitted to an Arduino board through serial communication, where they work as setpoints to PIcontrollers. 


[Main](Impl_Raspberry/APFControlador_Sensores.sce)



**Software: Arduino IDE, 
Hardware: Arduino Mega 2560**

The robot speed is measured by encoders coupled directly on the motor shaft, and the measured data is used by the odometry algorithm to compute the robot current position and to predict the positionthe robot will be one cycle later based on the current wheel speed setpoint applied to the PI. This predicted position is sent back to the Raspberry PI, what closes a cycle.

[Main](Impl_Arduino/PIDOdometria.ino)


Used Libraries:

[Control CC Motors](Impl_Arduino/Adafruit.zip)

[PID Controller](Impl_Arduino/PID.zip)

[Communication](Impl_Arduino/digitalWriteFastinterruptSafe.zip)

[Pins Control](Impl_Arduino/Utility.zip)


	
## Setup
To run this project on the robot:


1. Access Raspberry (SSH Communication)
- Conect both the computer and the RPI on the same network (Recommended: LUPA router)
- Access superuser via terminal:
```
$su
```
- Check the RPI IP address:
```
$nmap -sn YourIPAdress/24
```
- Connect via SSH (Standard RPI password is: raspberry)
```
$ssh @RpiIPAdress
```

2. Open Scilab on the Raspberry (The GUI requires too much processing)
```
$scilab-cli
```
3. Make sure the Arduino and the Raspberry are connected via cable.
4. Run:
[Main](Impl_Raspberry/APFControlador_Sensores.sce)
5. Enjoy it!

