// Include required libraries
#include <Wire.h>
#include <Ps3Controller.h>
#include <DFRobotDFPlayerMini.h>  // Cambio en la librería
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <FastLED.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

///////Luces Neopixel/////
#define NUM_LEDS 11   // Número total de Neopixels que estás utilizando
#define DATA_PIN 27    // Pin de datos de los Neopixel
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];
bool luces_encendidas = false;
unsigned long previousMillis = 0;
const long interval = 1000; // 1200 Intervalo de ejecución en milisegundos

//codigo del mando PS3
char MACaddress[] = "e8:6b:ea:df:03:de";

int player = 0;
int battery = 0;

int deadzone = 10;

// Define motor driver pins MX1508
#define IN3_PIN 19
#define IN4_PIN 23
#define IN1_PIN 5
#define IN2_PIN 18

#define IN5_PIN 4
#define IN6_PIN 2

// Define motor PWM Parameters
const int motorFreq = 1000;
const int motorResolution = 8;

// Definir canales para cada motor
const int motorAChannel1 = 3;
const int motorAChannel2 = 6;
const int motorBChannel1 = 4;
const int motorBChannel2 = 7;
// motor domo
const int motorCChannel1 = 9;
const int motorCChannel2 = 11;


// La velocidad del domo se reduce en la rotación del motor del domo mediante este valor multiplicador
float dome_speed = 0.75;  //0.75 Aumenta o disminuye para controlar velocidad domo, el valor máximo es 1,0

// Variables para los valores del motor PWM
int motorAPWM = 0;
int motorBPWM = 0;
int motorCPWM = 0;

// Variables para la dirección del motor - true=forward
bool motorDir = true;

// Variables para valores del joystick derecho
int rightX = 0;
int rightY = 0;

//Variable para almacenar salidas de velocidad
int speedX = 0;
int speedY = 0;
int speedZ = 0;

float output1; // Output from first channel
float output2; // Output from second channel
float theta; // conversion to polar coordinates
float radius; // conversion to polar coordinates
float rawLeft;
float rawRight;
float RawLeft;
float RawRight;

// Define parametros servo
#define frontarm1_MIN 190   //190 R2
#define frontarm1_MAX 370   //400
#define frontarm2_MIN 370   //370 L2
#define frontarm2_MAX 540   //590

// Define los valores mínimos y máximos para los servos
#define LIFTUP 420   // 418  +Nº mas alto, -Nº mas bajo para arriba
#define LIFTDOWN 226 // 223  +Nº mas alto, -Nº menos alto para abajo
#define TILTMIN 443  // 448 Valor para para ponerse recto +Nº mas recto, -Nº menos recto
#define TILTMAX 240  // 243 Valor para para dar inclinacion +Nº menos inclinacion -Nº mas inclinacion

// Definir los estados para el pie central y las piernas
#define CENTRE_LIFT_UP 1
#define CENTRE_LIFT_DOWN 0
#define LEGS_TO_CENTER 1
#define LEGS_TO_BACK 0

#define periscopio_MIN 405  // 405
#define periscopio_MAX 210  // 210
bool periscopioArriba = false;

int liftSpeed = 6; // 6 Velocidad de movimiento pie central + valor + lento
int tiltSpeed = 4; // 4 Velocidad de movimiento inclinacion + valor + lento

// Variables para mantener los estados del pie central y las piernas
int centreliftState = CENTRE_LIFT_DOWN;
int legtiltState = LEGS_TO_BACK;

// Variables for servo positions
int frontarm1 = 0;
int frontarm2 = 0;

// Valores del joystick izquierdo del controlador
int leftX;
int leftY;

// Pines usados para el bus I2C si es necesario
#define I2C_SDA 21
#define I2C_SCL 22

////////HOLOS////////
#define HOLO1_MIN 100 // 100  Top
#define HOLO1_MAX 500 // 550
#define HOLO2_MIN 50 // 150 Frente
#define HOLO2_MAX 600 // 550
#define HOLO3_MIN 150 // 250 Trasero
#define HOLO3_MAX 600 // 550

bool moving[] = {false, false, false};
unsigned long lastMoveTime[] = {0, 0, 0};
const unsigned long moveIntervals[] = {3000, 2500, 3000}; // Intervalos de tiempo entre movimientos en milisegundos (3, 4 y 5 segundos)

///////////sonidos//////////
// mySoftwareSerial(16, 17); // RX, TX
DFRobotDFPlayerMini musicPlayer;
bool volumeUpPressed = false; // Variable para rastrear si el botón de volumen hacia arriba está presionado
bool volumeDownPressed = false; // Variable para rastrear si el botón de volumen hacia abajo está presionado

///////////////////Elevar pie central/////////////////////////////////////////////
void moveLift(int targetPosition) {
  int currentPosition = pwm1.getPWM(3); // Obtener la posición actual del servo
  // Determinar la dirección del movimiento
  int step = (targetPosition > currentPosition) ? 1 : -1;
  // Realizar el movimiento gradual
  for (int pos = currentPosition; pos != targetPosition; pos += step) {
    pwm1.setPWM(3, 0, pos);
    delay(liftSpeed); // Ajustar la velocidad del movimiento (más tiempo = más lento)
  }
  // Establecer la posición final
  pwm1.setPWM(3, 0, targetPosition);
}
///////Inclinacion//////////
void moveTilt(int targetPosition) {
  int currentPosition = pwm1.getPWM(4); // Obtener la posición actual del servo
  // Determinar la dirección del movimiento
  int step = (targetPosition > currentPosition) ? 1 : -1;
  // Realizar el movimiento gradual
  for (int pos = currentPosition; pos != targetPosition; pos += step) {
    pwm1.setPWM(4, 0, pos);
    delay(tiltSpeed); // Ajustar la velocidad del movimiento (más tiempo = más lento)
  }

  // Establecer la posición final
  pwm1.setPWM(4, 0, targetPosition);
}

////movimiento Servos Holos
void moveServo(int servoIndex, int minAngle, int maxAngle) {
  int angle = random(minAngle, maxAngle);
  pwm2.setPWM(servoIndex, 0, angle); // Mover el servo al ángulo especificado
 // Serial.print("Empezar movimiento en el servo ");
  Serial.println(servoIndex);
}

void stopServo(int servoIndex, int minPulse, int maxPulse) {
  int neutralPulse = (minPulse + maxPulse) / 2;
  pwm2.setPWM(servoIndex, 0, neutralPulse); // Detener el servo
 // Serial.print("Detener movimiento en el servo ");
  Serial.println(servoIndex);
}

unsigned long previousMillis1 = 0;
unsigned long interval1 = 1300; // 1500 Intervalo de ejecución de efecto4_5_6 en milisegundos

void efecto1() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis;
    if (luces_encendidas) { // Verifica si las luces están encendidas
      leds[0] = CRGB(random(256), random(256), random(256)); // Color aleatorio entre rojo, verde y rosa
      FastLED.show();
      //Serial.println("efecto 1");
      interval1 = random(1000, 3200);
    }
  }
}

unsigned long previousMillis2_3 = 0;
unsigned long interval2_3 = 200; // 300 Intervalo de ejecución de efecto4_5_6 en milisegundos

void efecto2_3() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis2_3 >= interval2_3) {
    previousMillis2_3 = currentMillis;
    if (luces_encendidas) { // Verifica si las luces están encendidas
      // Generar números aleatorios entre 0 y 3 para determinar la combinación de colores
      int color1 = random(2); // 0 para azul, 1 para blanco
      int color2 = random(2); // 0 para azul, 1 para blanco
      
      // Asignar colores según los números aleatorios generados
      if (color1 == 0 && color2 == 0) {
        // Azul y Azul
        leds[1] = CRGB(0, 0, 255); // Azul
        leds[2] = CRGB(0, 0, 255); // Azul
      } else if (color1 == 0 && color2 == 1) {
        // Azul y Blanco
        leds[1] = CRGB(0, 0, 255); // Azul
        leds[2] = CRGB(255, 255, 255); // Blanco
      } else if (color1 == 1 && color2 == 0) {
        // Blanco y Azul
        leds[1] = CRGB(255, 255, 255); // Blanco
        leds[2] = CRGB(0, 0, 255); // Azul
      } else {
        // Blanco y Blanco
        leds[1] = CRGB(255, 255, 255); // Blanco
        leds[2] = CRGB(255, 255, 255); // Blanco
      }
      
      FastLED.show();
      //Serial.println("efecto 2_3");

      // Cambiar el intervalo a un valor aleatorio entre 500 ms y 3000 ms (3 segundos)
      interval2_3 = random(300, 1300); // Genera un número aleatorio entre 500 y 3000 (3001 excluido)
    }
  }
}


unsigned long previousMillis_4_5_6 = 0;
unsigned long interval_4_5_6 = 300; // 500 Intervalo de ejecución de efecto4_5_6 en milisegundos

void efecto4_5_6() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis_4_5_6 >= interval_4_5_6) {
    previousMillis_4_5_6 = currentMillis;
    if (luces_encendidas) {
      leds[3] = CRGB(random(156), random(256), random(256)); // Color aleatorio para el Neopixel 4
      leds[4] = CRGB(random(256), random(156), random(256)); // Color aleatorio para el Neopixel 5
      leds[5] = CRGB(random(256), random(256), random(156)); // Color aleatorio para el Neopixel 6
      FastLED.show();
     // Serial.println("efecto 4.5.6");
      interval_4_5_6 = random(300, 1200);
    }
  }
}

unsigned long previousMillis7 = 0;
unsigned long interval7 = 500; // Intervalo de ejecución de efecto7 en milisegundos

void efecto7() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis7 >= interval7) {
    previousMillis7 = currentMillis;
    if (luces_encendidas) {
      leds[6] = CRGB(random(256), random(256), random(256)); // Color aleatorio entre rojo, verde y rosa
      FastLED.show();
     // Serial.println("efecto 7");
      interval7 = random(1000, 3500);
    }
  }
}

unsigned long previousMillis8_9_10[] = {0, 0, 0};
unsigned long interval8_9_10[] = {200, 250, 280}; // Intervalos iniciales de ejecución en milisegundos

void efecto8_9_10() {
  unsigned long currentMillis = millis();
  for (int i = 7; i <= 9; ++i) {
  if (currentMillis - previousMillis8_9_10[i - 7] >= interval8_9_10[i - 7]) {
      previousMillis8_9_10[i - 7] = currentMillis;
      if (luces_encendidas) {
        // Toggle de encendido/apagado independiente para cada neopíxel
        if (random(2) == 1) {
          leds[i] = CRGB(random(256), random(256), random(256)); // Color aleatorio
        } else {
          leds[i] = CRGB::Black; // Apagar
        }
        FastLED.show();
        // Asignar un nuevo intervalo aleatorio para este neopíxel
        interval8_9_10[i - 7] = random(50, 250);
      }
    }
  }
}

unsigned long previousMillis11 = 0;
unsigned long interval11 = 500;

void efecto11() { 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis11 >= interval11) {
    previousMillis11 = currentMillis;
  if (periscopioArriba = true) { // Verificar si el periscopio está en "min" y las luces están encendidas
      leds[10] = CRGB(random(256), random(0), random(128)); // Color aleatorio entre rojo, verde y rosa
      FastLED.show();
      //Serial.println("efecto 11");
    }
  }
}

void efecto11_apagar() {
  leds[10] = CRGB::Black; // Apagar el LED número 10
  FastLED.show();
  //Serial.println("Efecto 11 apagado");
}

/////Luces Holo y paneles/////
void encenderLuces() {
  luces_encendidas = true;
  //Serial.println("iniciando luces");
}

void apagarLuces() {
    luces_encendidas = false;
    fill_solid(leds, NUM_LEDS, CRGB::Black); // Apagar todos los LEDs
    FastLED.show();
    if (!periscopioArriba) {
    efecto11_apagar();
  }      
}

// Función de devolución de llamada
void notify() {     
  // Obtener el valor del joystick izquierdo
  if ( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ) {
    leftX = (Ps3.data.analog.stick.lx);
    leftY = (Ps3.data.analog.stick.ly);
  }
  // Obtener el valor del joystick derecho
  if ( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ) {
    rightX = (Ps3.data.analog.stick.rx);
    rightY = (Ps3.data.analog.stick.ry);
  }

  speedX = map(leftX, 128, -128, -255, 255);
  speedY = map(leftY, 128, -128, -255, 255);
  speedZ = map(rightX, -128, 128, -255, 255);

// Ajuste de la velocidad basado en los estados centreliftState y legtiltState
  if (centreliftState == CENTRE_LIFT_UP && legtiltState == LEGS_TO_CENTER) {
      // Si los estados son CENTRE_LIFT_UP y LEGS_TO_CENTER, establecer la velocidad al 40%
      speedX *= 0.55;
      speedY *= 0.55;
      speedZ *= 0.65;  //valor para giro domo en 2 piernas
  }

  output1 = speedX;
  output2 = speedY;

  // Convierte ambos valores de palanca para gobernar de
  // coordenadas cartesianas a polares hipotenusa
  radius = sqrt(output1 * output1 + output2 * output2);
  // ángulo en radianes
  theta = acos(abs(output2) / radius);
  // atender a los valores NaN
  if (isnan(theta) == true) {
    theta = 0;
  }
  //ángulo en grados
  float angle = theta * 180 / 3.1415;
  float tcoeff = -1 + (angle / 90) * 2;
  float turn = tcoeff * abs(abs(output1) - abs(output2));
  turn = round(turn * 100) / 100;
  // Y el máximo de salida1 o salida2 es el movimiento
  float mov = max(abs(output1), abs(output2));
  // Primer y tercer cuadrante
  if ((output2 >= 0 && output1 >= 0) || (output2 < 0 && output1 < 0))
  {
    rawLeft = mov; rawRight = turn;
  }
  else
  {
    rawRight = mov; rawLeft = turn;
  }
  // Polaridad inversa
  if (output1 < 0) {
    rawLeft = 0 - rawLeft;
    rawRight = 0 - rawRight;
  }
  //Actualiza los valores
  RawLeft = rawLeft;
  RawRight = rawRight;


  // Asigna los valores al rango definido
  output1 = map(RawLeft, -255, 255, -255, 255);
  output2 = map(RawRight, -255, 255, -255, 255);
  output1 = constrain(output1, -255, 255);
  output2 = constrain(output2, -255, 255);
  motorAPWM = output1;
  motorBPWM = output2;

  //Establece las direcciones del motor
  if (motorAPWM < -deadzone) {
    motorAPWM = abs(motorAPWM);
    ledcWrite(motorAChannel2, motorAPWM);
    ledcWrite(motorAChannel1,  0);
  }
  else if (motorAPWM > deadzone) {
    ledcWrite(motorAChannel1, motorAPWM);
    ledcWrite(motorAChannel2,  0);
  }
  else {
    ledcWrite(motorAChannel1,  0);
    ledcWrite(motorAChannel2,  0);
  }

  if (motorBPWM < -deadzone) {
    motorBPWM = abs(motorBPWM);
    ledcWrite(motorBChannel2, motorBPWM);
    ledcWrite(motorBChannel1,  0);
  }
  else if (motorBPWM > deadzone) {
    ledcWrite(motorBChannel1, motorBPWM);
    ledcWrite(motorBChannel2,  0);
  }
  else {
    ledcWrite(motorBChannel1,  0);
    ledcWrite(motorBChannel2,  0);
  }

  // Print to Serial Monitor
  //  Serial.print("Xvalue= ");
  //  Serial.print(leftX);
  //  Serial.print(" Yvalue= ");
  //  Serial.print(leftY);
  //  Serial.print(" MotorA= ");
  //  Serial.print(motorAPWM);
  //  Serial.print(" MotorB= ");
  //  Serial.println(motorBPWM);

  ////////////////////////////////
  // Giro de cabeza

  speedZ = constrain(speedZ, -255, 255);
  motorCPWM = speedZ * dome_speed;

  if (motorCPWM <= -deadzone) {
    motorCPWM = abs(motorCPWM);
    ledcWrite(motorCChannel1, motorCPWM);
    ledcWrite(motorCChannel2,  0);
  }
  else if (motorCPWM >= deadzone) {
    ledcWrite(motorCChannel2, motorCPWM);
    ledcWrite(motorCChannel1,  0);
  }
  else {
    ledcWrite(motorCChannel1,  0);
    ledcWrite(motorCChannel2,  0);
  }

  //  Serial.print(" rightX= ");
  //  Serial.print(rightX);
  //  Serial.print(" head= ");
  //  Serial.print(motorCPWM);

  ///////////////////////////////

  //front arm 1
  int r2Value = Ps3.data.analog.button.r2;
  int frontarm1 = map(r2Value, 0, 255, frontarm1_MIN, frontarm1_MAX);
  pwm1.setPWM(0, 0, frontarm1);

  //front arm 2
  int l2Value = Ps3.data.analog.button.l2;
  int frontarm2 = map(l2Value, 0, 255, frontarm2_MIN, frontarm2_MAX);
  pwm1.setPWM(1, 0, frontarm2);

  // Print to Serial Monitor
//    Serial.print(" rightarm = ");
//    Serial.print(frontarm1);
//    Serial.print(" leftarm = ");
//    Serial.println(frontarm2);

  // Triángulo: elevar el pie central y mover las piernas al centro
  if (Ps3.event.button_down.triangle && centreliftState == CENTRE_LIFT_DOWN) {
    moveTilt(TILTMIN); // Inlinar TILTMIN a la posición mínima
    moveLift(LIFTUP); // Elevar pie LIFTUP a la posición máxima    
    centreliftState = CENTRE_LIFT_UP;
    legtiltState = LEGS_TO_CENTER;  
   // Serial.println("Subir pie central y piernas al centro");
  }

  // Botón X: bajar el pie central y mover las piernas hacia atrás
  if (Ps3.event.button_down.cross && centreliftState == CENTRE_LIFT_UP) {    
    // Reducir la velocidad para el movimiento hacia abajo
    int originalSpeed = liftSpeed;
    liftSpeed = 7; // 6 Puedes ajustar este valor para hacer el movimiento más lento
    moveLift(LIFTDOWN); // Bajar pie LIFTDOWN a la posición máxima
    liftSpeed = originalSpeed; // Restaurar la velocidad original
    moveTilt(TILTMAX); // Inclinar TILTMAX a la posición máxima
    centreliftState = CENTRE_LIFT_DOWN;
    legtiltState = LEGS_TO_BACK;
    //Serial.println("Bajar pie central y piernas hacia atrás");
  }
 
  //Boton L1 periscopio///  
  if (Ps3.event.button_down.l1) {
        // Si el periscopio está abajo, muévelo arriba
        if (!periscopioArriba) {
            pwm2.setPWM(0, 0, periscopio_MAX); // Mover el periscopio al valor máximo
            periscopioArriba = true; // Actualizar el estado del periscopio
            Serial.println(" periscopio_MAX ");            
            efecto11();           
        } else { // Si el periscopio está arriba, muévelo abajo
            pwm2.setPWM(0, 0, periscopio_MIN); // Mover el periscopio al valor mínimo
            periscopioArriba = false; // Actualizar el estado del periscopio
            Serial.println(" periscopio_MIN ");
            efecto11_apagar();
        }
              
  }
     //if( Ps3.event.button_up.l1 )
     //Serial.println("Released the left shoulder button");

  /// Movimiento holos//////  
    if (Ps3.event.button_down.r1) {
    //Serial.println("Botón presionado");
    for (int i = 0; i < 3; ++i) {
      if (!moving[i]) {
        moveServo(i + 1, (i == 0) ? HOLO1_MIN : ((i == 1) ? HOLO2_MIN : HOLO3_MIN), (i == 0) ? HOLO1_MAX : ((i == 1) ? HOLO2_MAX : HOLO3_MAX));
        moving[i] = true;
        lastMoveTime[i] = millis();
      } else {
        stopServo(i + 1, (i == 0) ? HOLO1_MIN : ((i == 1) ? HOLO2_MIN : HOLO3_MIN), (i == 0) ? HOLO1_MAX : ((i == 1) ? HOLO2_MAX : HOLO3_MAX));
        moving[i] = false;
      }
    }
  }
  
  ///////////////////////////////////////////
  //DFPlayermini sounds
  // Botón circular - reproducir sonido
  if ( Ps3.event.button_down.circle ) {
    Serial.println("Boton Circulo Pretado");
    int randomNumber = random(2, 46); // Genera un número aleatorio 
    musicPlayer.play(randomNumber);
  }
  
  // Botón cuadrado - reproducir sonido
  if ( Ps3.event.button_down.square ) {
    Serial.println("Boton Cuadrado Pretado"); 
    int randomNumber2 = random(47, 57); // Genera un número aleatorio 
    musicPlayer.play(randomNumber2);
  }
 
  // Volume control
  if (Ps3.data.button.up && !volumeUpPressed) { // Verifica si el botón de volumen hacia arriba está presionado y no estaba previamente presionado
    musicPlayer.volumeUp();
    //Serial.println("Subir Volumen");
    volumeUpPressed = true; // Establece el indicador de presión del botón de volumen hacia arriba como verdadero
  } else if (!Ps3.data.button.up && volumeUpPressed) { // Si el botón de volumen hacia arriba se suelta y estaba previamente presionado
    volumeUpPressed = false; // Establece el indicador de presión del botón de volumen hacia arriba como falso
  }

  if (Ps3.data.button.down && !volumeDownPressed) { // Verifica si el botón de volumen hacia abajo está presionado y no estaba previamente presionado
    musicPlayer.volumeDown();
    //Serial.println("Bajar Volumen");
    volumeDownPressed = true; // Establece el indicador de presión del botón de volumen hacia abajo como verdadero
  } else if (!Ps3.data.button.down && volumeDownPressed) { // Si el botón de volumen hacia abajo se suelta y estaba previamente presionado
    volumeDownPressed = false; // Establece el indicador de presión del botón de volumen hacia abajo como falso
  }

  // Luces holos y paneles - Botón Izquierdo Cruzeta     
     if (Ps3.event.button_down.right) {
        if (luces_encendidas) {
            apagarLuces();
           // Serial.println("Apagando luces...");
        } else {
            encenderLuces();
           // Serial.println("Encendiendo luces...");
        }
    }
    unsigned long currentMillis = millis();
    if (luces_encendidas) {
        // Verificar si es tiempo de ejecutar los efectos de iluminación
        if (currentMillis - previousMillis >= interval) {
            // Guardar el último tiempo de ejecución
            previousMillis = currentMillis;
            
            // Ejecutar los efectos de iluminación
            efecto1(); // Cambio de color aleatorio
            efecto2_3(); // Alternancia entre azul y blanco
            efecto4_5_6(); // Colores aleatorios
            efecto7(); // Cambio de color aleatorio con intervalo variable
            efecto8_9_10(); // Parpadeo de luces            
        }
    }
} 
      
/////////////////////////////////////////////////////////////////////////////
// On Connection function
void onConnect() {
  Serial.println("Connected.....");
  musicPlayer.play(58);
}
/////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);

  // Inicializar control PS3
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin(MACaddress);

  // Inicializar DFPlayerMini
  Serial2.begin(9600);  
  if (!musicPlayer.begin(Serial2)) { // Inicializar DFPlayer Mini
    while (true);
  }
  Serial.println("DFPlayerMini online.");

  ////para controlar luces//////////
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.show(); 

  // Setup PCA9685
  pwm1.begin();
  pwm1.setPWMFreq(50);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  servoSetup();
 
  // Set motor controller pins as outputs
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(IN5_PIN, OUTPUT);
  pinMode(IN6_PIN, OUTPUT);

  // Set initial state for motor controller pins
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(IN5_PIN, LOW);
  digitalWrite(IN6_PIN, LOW);

  // Set channel parameters for each motor
  ledcSetup(motorAChannel1, motorFreq, motorResolution);
  ledcSetup(motorAChannel2, motorFreq, motorResolution);
  ledcSetup(motorBChannel1, motorFreq, motorResolution);
  ledcSetup(motorBChannel2, motorFreq, motorResolution);
  ledcSetup(motorCChannel1, motorFreq, motorResolution);
  ledcSetup(motorCChannel2, motorFreq, motorResolution);

  // Attach motor PWM pins to corresponding channels
  ledcAttachPin(IN1_PIN, motorAChannel1);
  ledcAttachPin(IN2_PIN, motorAChannel2);
  ledcAttachPin(IN3_PIN, motorBChannel1);
  ledcAttachPin(IN4_PIN, motorBChannel2);
  ledcAttachPin(IN5_PIN, motorCChannel1);
  ledcAttachPin(IN6_PIN, motorCChannel2);

  // Turn off motors
  ledcWrite(motorAChannel1,  0);
  ledcWrite(motorAChannel2,  0);
  ledcWrite(motorBChannel1,  0);
  ledcWrite(motorBChannel2,  0);
  ledcWrite(motorCChannel1,  0);
  ledcWrite(motorCChannel2,  0);

  // Print to Serial Monitor
  Serial.println("Mini Droid..... :)");
  //Serial.println(MACaddress);
}

/////////////////////////////////////////////////////////////////////////////

void loop() {
  
  if (!Ps3.isConnected())
    return;
  else {
    Serial.println("Connect PS3 controller..... :)");
    //apagamos los motores
    ledcWrite(motorAChannel1,  0);
    ledcWrite(motorAChannel2,  0);
    ledcWrite(motorBChannel1,  0);
    ledcWrite(motorBChannel2,  0);
    ledcWrite(motorCChannel1,  0);
    ledcWrite(motorCChannel2,  0);
  }
     for (int i = 0; i < 3; ++i) {
    if (moving[i] && millis() - lastMoveTime[i] >= moveIntervals[i]) {
      moveServo(i + 1, (i == 0) ? HOLO1_MIN : ((i == 1) ? HOLO2_MIN : HOLO3_MIN), (i == 0) ? HOLO1_MAX : ((i == 1) ? HOLO2_MAX : HOLO3_MAX));
      lastMoveTime[i] = millis();
    }
  } 
  delay(1000);  
}

void servoSetup() {
/////////tabla 1////////  
  pwm1.setPWM(0, 0, frontarm1_MIN); //front arm 1
  pwm1.setPWM(1, 0, frontarm2_MIN); //front arm 2
  pwm1.setPWM(2, 0, 0); //
  pwm1.setPWM(3, 0, LIFTDOWN); //centre leg lift
  pwm1.setPWM(4, 0, TILTMAX);// Body Tilt

  
//////////tabla 2//////////
  pwm2.setPWM(0, 0, periscopio_MIN); // Configuración inicial del servo del periscopio
  pwm2.setPWM(1, 0, (HOLO1_MIN + HOLO1_MAX) / 2); // holo1 (canal 1)Top
  pwm2.setPWM(2, 0, (HOLO2_MIN + HOLO2_MAX) / 2); // holo2 (canal 2) Delantero
  pwm2.setPWM(3, 0, (HOLO3_MIN + HOLO3_MAX) / 2); // holo3 (canal 3)
}
