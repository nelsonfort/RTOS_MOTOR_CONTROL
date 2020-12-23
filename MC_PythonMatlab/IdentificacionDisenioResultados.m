clc, clear
close all
load('data')
%% 

%%%  OBTENIENDO RESPUESTA AL IMPULSO PARA IDENTIFICAR EL SISTEMA %%%
%% Leemos la curva que corresponde a un escalon de velocidad

verifyData = any(dataSpeedTime7(:,1),2);
dataOut = dataSpeedTime7(verifyData,:);
%- Se grafica
plot(dataOut(:,1),dataOut(:,2))
title('Pulsos de velocidad generados')
xlabel('Tiempo[ms]')
ylabel('Velocidad[rpm]')
grid on
figure
%- Vamos a realizar un zoom en la regíon de datos de interes
%- Y además vamos a superponerle el escalon de velocidad de referencia
plot(dataOut(1:2500,1),dataOut(1:2500,2))
stepRpse = dataOut(1:2500,:);
input = stepRpse;
input(1564:end,2) = 475; 
hold on
plot(input(:,1),input(:,2),'r')
title('Respuesta al escalon del motor')
xlabel('Tiempo[ms]')
ylabel('Velocidad[rpm]')
grid on
%- Las dos siguientes variables que son vectores con los datos del escalón
%- de referencia y la respuesta de velocidad obtenida es utilizado para
%- identificar el sistema. Como las mediciones fueron efectuadas con una
%- frecuencia de muestro de 1ms debe indicarse en el toolbox de
%- identificación
input = input(:,2);
stepRpse = stepRpse(:,2);

%- Teniendo el toolbox de identificación de sistemas instalado lo
%- ejecutamos con el comando "ident" y cargamos input y stepRpse.


%% *************   Diseño del controlador PID   ********************
%--------------------------------------------------------------------------
%---Control digital en base al modelo de motor de contínua con escobillas
%--------------------------------------------------------------------------
%   Vamos a desarrollar un sistema de control digital mediante el uso
%   de un control PID para obtener el siguiente desempeño del sistema:
%   
%   *   Tiempo de establecimiento < 2s
%   *   Sobrepico < 8%
%   *   Error de estado estacionario < 1%
%--------------------------------------------------------------------------
s = tf('s');
%- Dentro de 'sysIdent1.mat' se tiene el modelo obtenido mediante el
%- toolbox de identificación.
load('sysIdent1.mat')
%---------------------------------------
%---DC Motor Model en Laplace-TF
%---------------------------------------
s_DCMM = tf(tf1.Numerator, tf1.Denominator); 
zpk(s_DCMM)
%---------------------------------------
%---DC Motor Model en Z-TF
%---------------------------------------
%   Como estamos diseñando un sistema de control digital
%   debemos tener en cuenta que la frecuencia de muestreo 
%   del sistema sea considerablemente mayor que las frecuencias
%   de la planta a analizar (en nuestro caso el Motor). 
%   si analizamos los polos y ceros de la función de 
%   transferencia anterior podemos determinar que el
%   polo dominante se encuentra en 2 que aproximadamente da un
%   tiempo de establecimiento de 2 seguntos.
%   Por lo cual eligiendo un período de muestreo de 0.05, la 
%   frecuencia de muestreo es mucho mayor que la dinámica del 
%   sistema.
%
%   Para discretizar la planta vamos a utilizar la función c2d
%   donde le pasamos la función de transferencia en s, el período
%   de muestreo y el string 'zoh' que indica que vamos a utilizar 
%   el circuito zero-order-hold.
%---------------------------------------
Ts = 0.005;     % Mismo periodo de muestreo que el utilizado al obtener la 
                % respuesta al impulso
z_DCMM = c2d(s_DCMM,Ts,'zoh'); 
zpk(z_DCMM)
%---------------------------------------
%---Sistema a lazo cerrado 'solo motor'
%---------------------------------------
%   Ahora se analiza la respuesta del sistema a lazo cerrado sin
%   ningún controlador incorporado. Para poder graficar la respuesta
%   del sistema a un escalon cuando se tiene un ZoH se utiliza la 
%   función stairs que como su nombre lo indica realiza escalones
%   en base al período de muestreo.
%---------------------------------------
figure
sys_cl = feedback(z_DCMM,1);
[y,t] = step(sys_cl,0.5);
stairs(t,y);
grid on;
xlabel('Tiempo[s]')
ylabel('Velocidad[rad/s]')
title('Respuesta al escalon(simulando ZoH)')

%---------------------------------------
%---Implementando un PID
%---------------------------------------
%   Según el gráfico anterior, podemos ver que el tiempo de
%   establecimiento es de aproximadamente 0.3 segundos.
%   A su vez, el sistema posee un gran error de estado 
%   estacionario.
%   Para ajustar la dinámica del sistema en base a los
%   requerimientos vamos a diseñar un controlador PID.
%---------------------------------------

Kp = 20;
Ki = 100;
Kd = 0.1;
s_C = Kp + Ki/s + Kd*s;
z_C = c2d(s_C,Ts,'tustin'); %-- Transformación bilineal
zpk(z_C)
figure
rlocus(z_C)%-- Analizamos el lugar de raices del controlador
axis([-1.05 1.05 -1.05 1.05])
title('Lugar de raices del controlador PID')
%---------------------------------------
%---Agregando el PID al sistema de lazo cerrado
%---------------------------------------
figure
sys_cl = feedback(1/25*z_C*z_DCMM,1);
[y,t] = step(sys_cl,10);
stairs(t,y);
grid on;
xlabel('Tiempo[s]')
ylabel('Velocidad[rad/s]')
title('Respuesta al escalon(simulando ZoH) - Con controlador PID')
%---------------------------------------
%---Lugar de raices para ver que sucedió
%---------------------------------------
%   Como se puede ver en el gráfico temporal anterior,
%   el sistema a lazo cerrado para el controlador diseñado
%   es inestable.
%   Para determinar la causa de la inestabilidad utilizamos
%   el gráfico de lugar de raices.
%---------------------------------------
figure
rlocus(z_C*z_DCMM)
axis([-2 2 -2 2])
title('Lugar de raices de la planta + controlador PID')

%---------------------------------------
%---Solución propuesta
%---------------------------------------
%   Se puede ver que el controlador propuesto posee
%   un polo en -1. Para cualquier ganancia este polo
%   inestabiliza el sistema ya que se desplaza por fuera
%   del círculo de radio unitario.
%   Para resolver esta situación se propone incorporar un
%   polo de manera de cancelar el cero situado en -0.82.
%   De esta forma el sistema se vuelve estable para un rango
%   de frecuencias.
%---------------------------------------
figure
z = tf('z',Ts);
z_C2 = z_C/(z-0.3448);
rlocus(z_C2*z_DCMM)
axis([-1.05 1.05 -1.05 1.05])
title('Lugar de raices de la planta + controlador PID(modificado)')

%---------------------------------------
%---Ajustamos el sistema con la ganancia elegida
%---------------------------------------
%   Seleccionando con el cursor diferentes posiciones de
%   los polos a medida que cambia la ganancia se obtienen
%   parámetros relacionados a la dinámica del sistema.
%   Se debe elegir el que se considera más adecuado en base
%   a los requerimientos indicados.
%---------------------------------------
figure
sys_cl = feedback(0.1*z_C2*z_DCMM,1);
[y,t] = step(sys_cl,0.5);
stairs(t,y);
grid on;
xlabel('Tiempo[s]')
ylabel('Velocidad[rad/s]')
title('Respuesta al escalon(simulando ZoH) - Con controlador PID(modificado)')

%---------------------------------------
%--- Metodo 2. haciendo uso de 'pidtune'
%---------------------------------------
%   Otro método de encontrar las ganancias del PID es haciendo uso de 
%   'pidtuneOptions' donde podemos setear parámetros de diseño del PID
%   que luego será diseñado mediante 'pidtune' dandole como parametro de 
%   entrada el modelo de la planta en tiempo discreto, el tipo de
%   controlador que deseamos utilizar y las opciones seleccionadas
%   previamente.
%   En primera instancia como el modelo pid daba condiciones inestables
%   debido a las características de la planta, vamos a pasar directamente
%   al modelo 'pidf' que consiste en la incorporación de un filtro de
%   primer orden en la parte derivativa del pid.
%---------------------------------------
opts = pidtuneOptions('PhaseMargin',45,'DesignFocus','balanced');
[z_C3,info] = pidtune(z_DCMM,'pidf',opts);
Kp = z_C3.Kp;
Ki = z_C3.Ki;
Kd = z_C3.Kd;

figure
rlocus(z_C3)%-- Analizamos el lugar de raices del controlador
axis([-1.05 1.05 -1.05 1.05])
title('Lugar de raices del controlador PIDF')
KK = 1
figure
sys_c3 = feedback(KK*z_C3*z_DCMM,1);
%- Se puede ver en el gráfico generado que el controlador encontrado es
%- mucho más rápido que el diseñado de forma empírica por otro lado el
%- sobrepico se encuentra dentro de los valores de diseño por lo cual se
%- implmenta éste controlador en el microcontrolador.
[y,t] = step(sys_c3,0.5);
stairs(t,y);
grid on;
xlabel('Tiempo[s]')
ylabel('Velocidad[rad/s]')
title('Respuesta al escalon - PID + Filtro de 1er ord en D')
figure
rlocus(KK*z_C3*z_DCMM)
%- Con el resultado obtenido de la siquiente ecuación se ontiene la
%- ecuación en diferencias que se va a implementar en el microcontrolador.
zpk(z_C3)
%------------------     Controlador obtenido     ----------------------
%-      y[n] = 0.599*x[n] + 0.1553*x[n-1] - 0.444*x[n-2] 
%-                        + 1.296*y[n-1] - 0.296*y[n-2]
%----------------------------------------------------------------------
%%

%%% --- RESULTADOS OBTENIDOS --- %%%
%% Lazo cerrado sin PID
%- Sistema a lazo cerrado sin controlador. Se puede ver el gran error de
%- estado estacionario obtenido y el comportamiento subamortiguado de la
%- planta.
A = importdata('speedVStimeSinPID.csv');
speedVStimeSinPID = A.data;
figure
plot(speedVStimeSinPID(:,1),speedVStimeSinPID(:,2),'r',speedVStimeSinPID(:,1),speedVStimeSinPID(:,3),'b');
title('Lazo cerrado sin PID - K=1')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on
figure
ini= 2500;
fin= 4000;
plot(speedVStimeSinPID(ini:fin,1),speedVStimeSinPID(ini:fin,2),'r',speedVStimeSinPID(ini:fin,1),speedVStimeSinPID(ini:fin,3),'b');
title('Lazo cerrado sin PID - K=1')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on

%% PID Check behavior for differents values of gain.
%- Analizamos el comportamiento del sistema con el PID diseñado teniendo en
%- cuenta una ganancia adicional K para ajustar los sobrepicos generados
%- por el controlador.
%- Ganancia K = 1
A = importdata('speedVStime3.csv');
speedVStime3 = A.data;
plot(speedVStime3(:,1),speedVStime3(:,2),'r',speedVStime3(:,1),speedVStime3(:,3),'b');
title('Motor controlado con PID - K=1')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on

%- Ganancia K = 0.7
figure
A = importdata('speedVStime4.csv');
speedVStime4 = A.data;
plot(speedVStime4(10:end,1),speedVStime4(10:end,2),'r',speedVStime4(10:end,1),speedVStime4(10:end,3),'b');
title('Motor controlado con PID - K=0.7')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on

%- Ganancia K = 0.5
figure
A = importdata('speedVStime5.csv');
speedVStime5 = A.data;
plot(speedVStime5(:,1),speedVStime5(:,2),'r',speedVStime5(:,1),speedVStime5(:,3),'b');
title('Motor controlado con PID - K=0.5')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on
figure
%- Como para la ganancia de 0.7 se obtuvieron mejores resultados (teniendo 
%- en cuenta los sobrepicos)
%- vamos a realizarle zoom a la señal para ver más en detalle
ini= 2500;
fin= 4000;
plot(speedVStime4(ini:fin,1),speedVStime4(ini:fin,2),'r',speedVStime4(ini:fin,1),speedVStime4(ini:fin,3),'b');
title('Motor controlado con PID - K=0.7')
xlabel('Tiempo[s]')
ylabel('Velocidad[rpm]')
grid on
%---------------- Cambio de velocidad de 50rpm --------------------
%- Los resultados obtenidos demuestran un comportamiento con un sobrepico
%- seguido de una serie de oscilaciones. De forma gráfica se determinaron 
%- las siguientes características:
%   *   Sobrepico < 5.5%.
%   *   Tiempo de establecimiento < 200ms para un error de estado
%       estacionario < 5%. 
%   *   Tiempo de establecimiento < 1.5s para un error de estado
%       estacionario < 1%. 
%   *   Tiempo de subida de 50ms.
%   *   Frecuencia de la oscilacion 2Hz.

