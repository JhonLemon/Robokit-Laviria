/*
 * ======================================================================================
 * CONTROL DE MOTORES DC 
 * ======================================================================================
 * PINOUT UTILIZADO (ARDUINO NANO / DRIVER L298N):
 * -------------------------------------------------------------------------------------
 * Motor derecho: 
 * C Rojo -> OUT3 L298N
 * C Negro -> GND Encoder
 * C Amarillo -> PIN Digital 5 (canal B encoder)
 * C Verde -> PIN Digital 3(canal A de encoder / pin interrupcion)
 * C Azul -> +5V Encoder
 * C Blanco -> OUT 4 L298N
 * 
 * Motor Izquierdo:
 * C Rojo -> OUT1 L298N
 * C Negro -> GND Encoder
 * C Amarillo -> PIN Digital 4 (canal B encoder)
 * C Verde -> PIN Digital 2(canal A de encoder / pin interrupcion)
 * C Azul -> +5V Encoder
 * C Blanco -> OUT 2 L298N
 * 
 *
 * COMANDOS DE CONTROL (MONITOR A 57600 BAUDOS):
 * --------------------------------------------------------------------------------------
 * m <rpm_izq> <rpm_der> : CONTROL PID -> de -120 a 120 rpm -> kp=4-0, ki=0.7, kd=0
 * o <pwm_izq> <pwm_der> : MODO "RAW"(Open Loop)-> de -255 a 255
 * s                     : STOP -> Frena totalmente los motores y apaga el PID
 * e                     : ENCODERS -> Imprime los ticks acumulados
 * r                     : RESET -> Pone contadores de ticks a 0 
 * u <Kp> <Kd> <Ki>      : SINTONIZACION -> Actualiza las constantes del PID en simulacion
 * ======================================================================================
 * 
 * NOTAS:
 * EL VALOR MAXIMO ALCANZADO POR LOS MOTORES ES DE 120 RPM A UN VOLTAJE DE 13.28 V
 * PARA OPERAR AL VALOR NOMINAL DE 12V LA VELOCIDAD PROMEDIO REGISTRADA NO EXCEDE DE LOS 110 RPM
 */

// --- DEPURACION GRAFICA CONTROL PID
//Dejar como false, a menos que quiera comprobarse por el serial plotter
//el seguimiento del control pid de forma grafica
#define D_PLOTTER false

// --- PARAMETROS DE LOS MOTORES
//Segun el datasheet por revolucion de eje: 11 ticks
//Caja reductora 45:1
//ticks por revolucion = 11*45= 495
#define ENC_TICKS_POR_REV 495.0

// --- FRECUENCIA PID
//Establecemos una frecuencia de 30 Hz para "corregir" el error en el control PID
//el control recolectara los ticks con sus conversiones y los actualizara 30 veces por segundo 
//es una tasa promedio ideal, se recomienda no modificar
#define PID_FREC 30
const int PID_INTERVAL= 1000/PID_FREC; //conversion a ms -> aprox sera cada 33 ms entre calculo

// --- PINOUT(A CONSIDERAR)
//Tomar en cuenta que los pines 2 y 3 del arduino nano son de interrupcion, se debe procurar
//que al menos uno de ellos se use para cada motor y no ambos en uno solo
#define ENC_IZQ_A 2 
#define ENC_IZQ_B 4
#define ENC_DER_A 3
#define ENC_DER_B 5

// --- DRIVER L298N
//Desconectar el puente de los extremos para el envio de la señal PWM
//IN1/IN2 controlan la direccion de giro en cada caso
#define MOTOR_IZQ_PWM 6
#define MOTOR_IZQ_IN1 7
#define MOTOR_IZQ_IN2 8

#define MOTOR_DER_PWM 9
#define MOTOR_DER_IN3 10
#define MOTOR_DER_IN4 11

// --- VARIABLES DE INTERRUPCION
//variables volatiles para conteo y funciones de interrupcion ISR (interruption service routine)
volatile long IZQ_enc_pos = 0L;
volatile long DER_enc_pos= 0L;

// --- CONSTANTES PID
/* Kp: reaccion proporcional al error actual
 * ki: acumulacion de error para mayor acercamiernto 
 * kd: prediccion y suavizado (preferible no utilizarse)
 * 
 * variables double por el calculo que desempeñan
 */
double Kp= 4.0; 
double Ki= 0.7;
double Kd= 0.0; 

// --- TIEMPO (para Multitask)
//Marca de tiempo de la última vez se calcula el PID
unsigned long lastPID_t = 0; 

// --- Variables de OBJETIVO Y ENTRADA
double target_rpm_IZQ= 0; 
double target_rpm_DER= 0;
double input_rpm_IZQ= 0;  
double input_rpm_DER= 0;

// --- Variables INTERNAS DEL PID
//Posición de los encoders en el ciclo anterior 
long prev_enc_IZQ= 0;      
long prev_enc_DER= 0;
//Acumulador de error (ki)
double error_integral_IZQ= 0;
double error_integral_DER= 0;
//Error anterior (kd)
double last_error_IZQ= 0;
double last_error_DER= 0;

// --- SALIDA A LOS MOTORES
int pwm_IZQ= 0;
int pwm_DER= 0;
//Bandera de seguridad
bool use_pid = false; 


// ======================= SETUP =========================

void setup(){
  Serial.begin(57600); //Misma tasa de baudios en ROS

  //Configuracion del pinout
  pinMode(MOTOR_IZQ_PWM,OUTPUT);
  pinMode(MOTOR_IZQ_IN1,OUTPUT);
  pinMode(MOTOR_IZQ_IN2,OUTPUT);
  
  pinMode(MOTOR_DER_PWM,OUTPUT);
  pinMode(MOTOR_DER_IN3,OUTPUT);
  pinMode(MOTOR_DER_IN4,OUTPUT);

  //Entradas pull up para evitar ruido electrico ante desconexion
  pinMode(ENC_IZQ_A,INPUT_PULLUP);
  pinMode(ENC_IZQ_B,INPUT_PULLUP);
  
  pinMode(ENC_DER_A,INPUT_PULLUP);
  pinMode(ENC_DER_B,INPUT_PULLUP);

  //Funciones de interrupcion (se realizan sin importar el proceso que se este ejecutando)
  //sintaxis attachInterrupt(digitalPinToInterrupt(pin), ISR, mode) 
  //tomamos el flanco de subida para evitar caer en interrupciones dobles con CHANGE
  //EL USO DE RISING CONDICIONA LA LOGICA DE GIRO
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), doEncoderIzq, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_A), doEncoderDer, RISING);

  // Estado DEFAULT: Motores detenidos
  stopMotors();

  //
  if(!D_PLOTTER){
    Serial.println("Arduino Motor Controller Ready");
    Serial.println(" --- Comandos de operacion ---");
    Serial.println("m - <rpm_izq> <rpm_der>");
    Serial.println("o - <pwm_izq> <pwm_der>");
    Serial.println("u - <Kp> <Kd> <Ki>");
    Serial.println("r - reset");
    Serial.println("e - encoders");
    Serial.println("s - stop");


    
  }
  
}

// ======================= LOOP =========================

void loop(){
  // --Tarea 1: Lectura serial
  if(Serial.available()>0){
    //"Arduino library for parsing commands of the form COMMAND_NAME ARG1 ARG2"
    //ideal para el formato de los comandos establecidos
    //llamamos a la funcion para aplicar el metodo de parsing
    parseCommand(Serial.read());
  }

  // --Tarea2: Control PID
  //uso de millis como cronometros para repetir el ciclo de PID_INTERVAL (cada 33ms)
  //permite al arduino seguir escuchando el Serial Port mientras espera
  unsigned long currentMillis = millis();

  if(currentMillis-lastPID_t >= PID_INTERVAL){
    lastPID_t=currentMillis;
    
    //Calculo de ticks en el PID_INTERVAL(33ms)
    long current_ticks_IZQ= IZQ_enc_pos - prev_enc_IZQ;
    long current_ticks_DER= DER_enc_pos - prev_enc_DER;

    //Actualizamos para la proxima resta
    prev_enc_IZQ= IZQ_enc_pos;
    prev_enc_DER= DER_enc_pos;

    // ---  CONVERSION DE TICKS A RPM
    //(Ticks *Frecuencia de muestreo*60seg)/Ticks por revolucion

    input_rpm_IZQ= (current_ticks_IZQ*PID_FREC*60.0)/ENC_TICKS_POR_REV;
    input_rpm_DER= (current_ticks_DER*PID_FREC*60.0)/ENC_TICKS_POR_REV;

    // ---EJECUCION DE PID (SI LA BANDERA ESTA ACTIVA)
    if(use_pid){
      
      pwm_IZQ= calculePID(target_rpm_IZQ, input_rpm_IZQ, error_integral_IZQ, last_error_IZQ);
      pwm_DER= calculePID(target_rpm_DER, input_rpm_DER, error_integral_DER, last_error_DER);

      //Calculado el nuvel de voltaje es enviado a los motores
      //0 y 1 --> M_IZQ y M_DER
      setMotorSpeed(0, pwm_IZQ);
      setMotorSpeed(1, pwm_DER);  
      }
      
      // --- DEBUGGER POR SERIAL PLOTTER (marcar como true la definicion)
    if (D_PLOTTER) {
      //solo es para motor izquierdo (se probo enviando la misma velocidad a ambos)
      Serial.print("Target:"); Serial.print(target_rpm_IZQ);
      Serial.print(",Actual:"); Serial.print(input_rpm_IZQ);
      Serial.print(",PWM:"); Serial.println(pwm_IZQ / 2); 
      }        
  }
}

// ======================= PID =========================
//Esencialmente retorna el valor equivalente del error (de 0 a 255)
//uso de paso por referencia para usar una copia del valor de integral y last error sin modificar 
//su declaracion previa
int calculePID(double target, double current, double &integral, double &last_error){
  
  // 1-Calculo del error
  double error= target - current;

  // 2- Termino ki (acumula el error en el tiempo y forza a llegar al target)
  integral +=error;
  //limitacion al termino de la integracion en el intervalo establecido (y sobrecarga de los motores)
  integral= constrain(integral,-1000,1000);

  // 3- Termino kd (predice el comportamiento frenando el cambio brusco del error, no se usara de todas formas)
  double derivada = error-last_error;

  last_error= error;//Guardamos el valor para el siguiente intervalo

  // 4- Sumatoria (discretizada en los pasos anteriores y dentro del espacio del tiempo (t))
  double output= (Kp*error) + (Ki*integral) +(Kd*derivada);

  // 5- Limitacion del PWM (-255 a 255) ademas de la conversion a entero
  return constrain((int)output, -255,255);
 
}

// ======================= COMANDOS =========================
void parseCommand(char cmd){

  // COMANDO 'm' uso del PID en RPM
  if(cmd== 'm'){
    float v1= Serial.parseFloat();
    float v2= Serial.parseFloat();
    target_rpm_IZQ= v1;
    target_rpm_DER= v2;

    use_pid= true;
    //IMPORTANTE: Reset de variables PID para evitar saltos bruscos en el motor lo mas posible
    error_integral_IZQ=0;
    error_integral_DER=0;
  
    last_error_IZQ=0;
    last_error_DER=0;
  
    //innecesario para ambos
    if(!D_PLOTTER){
      Serial.print("PID TARGET -> L:"); Serial.print(target_rpm_IZQ);
      Serial.print(" R:"); Serial.println(target_rpm_DER);
    }
  }
  
  //COMANDO 'o' (open loop) uso del PWM - SOLO VALORES ENTEROS
  else if (cmd=='o'){
    int p1= Serial.parseInt();
    int p2= Serial.parseInt();
    
    use_pid= false;
    
    setMotorSpeed(0, p1);
    setMotorSpeed(1, p2);

    //innecesario excepto para ver la equivalencia del RPM al consumo energetico por medio del PWM
    if(!D_PLOTTER) {
      Serial.print("PWM MANUAL -> L:"); Serial.print(p1);
      Serial.print(" R:"); Serial.println(p2);
    } 
  }


  //COMANDO 's' paro total
  else if(cmd=='s'){
    // Frena y apaga PID
    stopMotors(); 
    if(!D_PLOTTER) Serial.println("MOTORES STOP");
  }

  //COMANDO 'u' actualiza constantes PID (update)
  else if(cmd== 'u'){
    Kp = Serial.parseFloat();
    Kd = Serial.parseFloat();
    Ki = Serial.parseFloat();
    
    if(!D_PLOTTER) {
      Serial.print("PID UPDATE -> Kp:"); Serial.print(Kp);
      Serial.print(" Kd:"); Serial.print(Kd);
      Serial.print(" Ki:"); Serial.println(Ki);
    }

    }
  
  //COMANDO 'r' Reset del numero de ticks (tambien detiene los motores)
  else if(cmd=='r'){
   
    IZQ_enc_pos = 0; DER_enc_pos = 0;
    target_rpm_IZQ = 0; target_rpm_DER = 0;
    stopMotors();
    
    Serial.println("RESET OK");
  }

  //COMANDO 'e' lectura de los ticks de Encoders
  else if (cmd== 'e'){
    
    Serial.print("ENC: "); Serial.print(IZQ_enc_pos);
    Serial.print(" "); Serial.println(DER_enc_pos);    
  }

  //LIMPIEZA DEL BUFFER SERIAL   
  //Serial peek inspecciona el dato mas no lo lee
  while(Serial.available() > 0 && Serial.peek() <= ' ') Serial.read();
     
}

// ======================= DRIVER L298N =========================
void setMotorSpeed(int motor, int speeed) {
  int pwmPin, in1, in2;
  
  //Seleccion de pines segun el motor (0 -> Izq, 1 -> Der)
  if (motor== 0){ 
    pwmPin = MOTOR_IZQ_PWM; in1= MOTOR_IZQ_IN1; in2= MOTOR_IZQ_IN2;
    }
  else {
    pwmPin = MOTOR_DER_PWM; in1 = MOTOR_DER_IN3; in2 = MOTOR_DER_IN4; 
    }

  //Lógica de Puente H para la dirección
  if (speeed > 0) {
    //Forward
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
  } 
  else if (speeed < 0) {
    //Backward
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
  } 
  else {
    //STOP
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);  
  }
  
  //Enviamos la potencia (ES UN VALOR PWM POSITIVO!!)
  analogWrite(pwmPin, abs(speeed));
}

// ======================= STOP/DEFAULT =========================
void stopMotors() {
  setMotorSpeed(0, 0); 
  setMotorSpeed(1, 0);
  // IMPORTANTE Evita que el PID intente corregir un "error" inexistente
  use_pid = false; 
}

// ======================= RUTINAS DE INTERRUPCION (ISR) =========================
// Estas funciones se ejecutan automáticamente por HARDWARE
void doEncoderIzq() {
  // Leemos el canal B para determinar la dirección del giro
  if (digitalRead(ENC_IZQ_B) == LOW) IZQ_enc_pos++;
  else IZQ_enc_pos--;
}
void doEncoderDer() {
  if (digitalRead(ENC_DER_B) == LOW) DER_enc_pos++;
  else DER_enc_pos--;
}
