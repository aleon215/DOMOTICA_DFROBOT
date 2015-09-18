
/* 
Domotica v1
Fecha: 2015-04-12
Autor: Alejandro Leon
Email: alejandroleon215@gmail.com

Descripcion: 

Obtiene valores de temperatura y humedad desde el sensor DHT11 y los envia a Ubidots para registro y graficacion.
La tira de LEDS se enciende segun el modo en el que se ejecute la aplicacion. Los modos se cambian mediante un mando de infrarojos.

Modo 0: No se enciende la tira de LEDS
Modo 1: Muestra colores en base a la temperatura
Modo 2: Cambia colores en base a sonidos captados por el sensor de sonido
Modo 3: Cambia los colores en base al sensor de distancia

Sensores: 

1: Lector de infrarojos
2: Sensor de temperatura y humedad DHT11
3: Sensor de sonido
4: Sensor de distancia

*/

// Includes

#include <IRremote.h>
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Definicion de pines

#define IR_PIN 2     // Sensor de infrarojos
#define DHTPIN 6     // Sensor DHT11 de temperatura y humedad
#define BLUEPIN 9    // PWM
#define REDPIN 10    // PWM
#define GREENPIN 11  // PWM
#define MIC_PIN A0   // Microfono
#define DFROBOT3 3   // GSM
#define DFROBOT4 4   // GSM
#define DFROBOT5 5   // GSM
#define DEBUG_RX A2  // Software serial RX for debuging
#define DEBUG_TX A1  // Software serial TX for debuging

// Debug serial

SoftwareSerial mySerial(DEBUG_RX, DEBUG_TX); // RX, TX

// Variables y constantes

#define READS 3 // Numero de lecturas del sensor de sonido para generar un promedio

int LEDSMode = 0; // 0=No luz, 1=Temp, 2=Sonido, 3=Distancia
int currentRead = 0;
int values[READS];

// Temporizador para apagar LEDS si no hay sonido
unsigned long timer_old_sonido;
int tiempo_maximo_sonido = 3; // Tiempo en segundos para leer sensores y enviar a ubidots

// Temporizador para leer temp y humedad y enviar a Ubidots
unsigned long timer_old_ubidots;
unsigned long tiempo_maximo_ubidots = 300; // Tiempo en segundos para leer sensores y enviar a ubidots

// Temporizador para leer sensores y mostrar en LCD
unsigned long timer_old_sensors;
int tiempo_maximo_sensors = 5; // Tiempo en segundos para leer sensores

// Temporizador para leer sensores y cambiar luces LEDS en base a temperatura
unsigned long timer_old_LEDS;
int tiempo_maximo_LEDS = 1; // Tiempo en segundos para leer sensores

// DHT

#define DHTTYPE DHT11   // DHT 11 

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Pantalla LCD

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Sensor IR

IRrecv irrecv(IR_PIN);
decode_results IR_resultados;

// Ubidots authentification token
extern String token;

// ------------ SETUP -------------

void setup() {

   mySerial.begin(9600);
   mySerial.println(F("Iniciando software serial"));
    
   Serial.begin(9600);
   mySerial.println(F("Iniciando modem serial"));
  
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  // Arrancamos el sensor IR
  irrecv.enableIRIn();
  
  // Mostramos por defecto el modo 0
  mySerial.print(F("LEDS mode: "));
  mySerial.println(LEDSMode);
  
  // LCD setup
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight 
  
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on 
  
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 1 on line 0
  lcd.print(F("DOMOTICA v1.0"));
  lcd.setCursor(0,1);
  lcd.print(F("INICIANDO"));
  
  // DFRobot GSM modem setup
  
  pinMode(DFROBOT3,OUTPUT);
  pinMode(DFROBOT4,OUTPUT);
  pinMode(DFROBOT5,OUTPUT);
  
  TurnOnModem(); // Encendemos modem
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("MODEM OK"));
  lcd.setCursor(0,1);
  lcd.print(F("LEYENDO DATOS"));
  delay(2000);
  
  // Leemos los primeros datos para mostrar en el LCD
  
  // Leer temperatura
  float temp = dht.readTemperature();
  
  // Leer humedad
  float hum = dht.readHumidity();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(temp) || isnan(hum)) {
          mySerial.println(F("Failed to read from DHT sensor!"));
          return;
        }
  
    PrintSeconds();
    mySerial.print(F("Temp: "));
    mySerial.println(temp);
  
    PrintSeconds();
    mySerial.print(F("Humedad: "));
    mySerial.println(hum);
  
    // Print to LCD screen
    printLCD(temp,hum);
    
    TurnOffModem();
    
    // LLenamos las variables de temporizador
    timer_old_ubidots = millis();
    timer_old_sensors = millis();
    timer_old_LEDS = millis();
}

// ------------ LOOP -------------

void loop() {
  
  // Leer infrarojos y establecer el modo si se recibe el valor 77E1C062
  leerIR();
  
  // En todos los modos se leen los sensores y se envian los datos a Ubidots
  leerSensores();
  
  switch(LEDSMode) {
    case 0:
      //LEDS apagados
      apagarLedsTemp();
      break;
    case 1:
      //LEDS muestran temperatura
      encenderLedsTemp();
      break;
    /*
    case 2:
      //LEDS con sonido
      encenderLedsSonido();
      break;
    */
  }
}


// ------------ FUNCIONES -------------

void printLCD(float temp, float hum) {

    lcd.clear();
    lcd.setCursor(0,0); //Start at character 1 on line 0
    lcd.print(F("T:")); lcd.print(temp,0); lcd.print((char)223); lcd.print(F("C "));
    lcd.print(F("H:")); lcd.print(hum,0); lcd.print(F("%"));
    printMode(LEDSMode);
    lcd.setCursor(0,1);
    lcd.print(F("Enviar: "));
}

void setLedColor(int n) {
  
  int randNumberR;
  int randNumberB;
  int randNumberG;
  randNumberR = random(255);
  randNumberB = random(255);
  randNumberG = random(255);
  
  analogWrite(REDPIN, randNumberR);
  analogWrite(BLUEPIN, randNumberB);
  analogWrite(GREENPIN, randNumberG);
 }
 
void TurnOnModem()
{
  
  uint8_t answer=0;
  PrintSeconds();
  mySerial.print(F("Turning on Modem"));
  
    if ((sendATcommand("AT","OK",4000)) == 0) {
      // Si AT no reponde con OK (response=0), entonces el modem esta apagado y procedemos a encender
      digitalWrite(5,HIGH);
      delay(1000);
      digitalWrite(5,LOW);
      delay(1000);
    }
  
  digitalWrite(3,LOW);//enable GSM TX、RX
  digitalWrite(4,HIGH);//disable GPS TX、RX
  delay(5000);
  
  // waits for an answer from the module
  /*
  while(answer == 0) {     // Send AT every two seconds and wait for the answer
    answer = sendATcommand("AT", "OK", 2000);    
   }
  */
}

void TurnOffModem()
{
  PrintSeconds();
  mySerial.print(F("Turning off modem"));
  
    if ((sendATcommand("AT","OK",4000)) == 1) {
      // Si AT si reponde con OK, entonces el modem esta encendido y procedemos a apagar
      digitalWrite(5,HIGH);
      delay(1000);
      digitalWrite(5,LOW);
      delay(1000);
    }
}

void PrintSeconds()
{
  float tiempo = millis()/60000.00;
  mySerial.print(tiempo,2);
  mySerial.print(F(" mins: "));
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);    // Initialize the string
    
    delay(100);
    
    while(Serial.available() > 0) Serial.read();    // Clean the input buffer
    
    Serial.println(ATcommand);    // Send the AT command 


    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(Serial.available() != 0){    
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = Serial.read();
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL)    
            {
                answer = 1;
            }
        }
         // Waits for the asnwer with time out
    }while((answer == 0) && ((millis() - previous) < timeout)); 
  
  //PrintSeconds();
  mySerial.print(F(" sendATcommand request: "));
  mySerial.println(ATcommand);
  
  //PrintSeconds();
  mySerial.print(F(" sendATcommand response: "));  
  mySerial.println(response);

  return answer;
}



void Send2ubidots(String value, String variableID, String tokenID)
{

  int num;
  String le;
  String var;
  String variableIDStr;
  String tokenIDStr;
  
  var = "{\"value\":"+ value + "}";           //value is the sensor value
  num = var.length();
  le = String(num);                           //this is to calcule the length of var
  variableIDStr = "POST /api/v1.6/variables/" + variableID + "/values HTTP/1.1";
  tokenIDStr = "X-Auth-Token: " + tokenID;
  
  for(int i = 0;i<7;i++)
  {
    mySerial.println("AT+CGATT?");    //this is made repeatedly because it is unstable
    Serial.println("AT+CGATT?");
    delay(2000);
    PrintSeconds();
    ShowSerialData();
  }
  
  PrintSeconds();
  mySerial.println("AT+CIPSPRT=0");
  Serial.println("AT+CIPSPRT=0");
  delay(3000);
  PrintSeconds();
  ShowSerialData();

  PrintSeconds();
  mySerial.println(F("Start the connection to Ubidots"));
  
  PrintSeconds();
  mySerial.println(F("AT+CIPSTART=\"TCP\",\"things.ubidots.com\",\"80\""));
  Serial.println(F("AT+CIPSTART=\"TCP\",\"things.ubidots.com\",\"80\""));
  PrintSeconds();
  delay(3000);
  PrintSeconds();
  ShowSerialData();
  
  PrintSeconds();
  mySerial.println(F("Begin to send data to the remote server"));

  PrintSeconds();
  Serial.println(F("AT+CIPSEND"));
  mySerial.println(F("AT+CIPSEND"));
  delay(3000);
  PrintSeconds();
  ShowSerialData();
  
  PrintSeconds();
  mySerial.println(F("Sending request to Ubidots"));
  
  Serial.println(variableIDStr);           // Replace "xxx..." with your variable ID
  mySerial.println(variableIDStr);
  delay(100);
  
  Serial.println(F("Content-Type: application/json"));
  mySerial.println(F("Content-Type: application/json"));
  delay(100);
  
  Serial.println("Content-Length: "+le);
  mySerial.println("Content-Length: "+le);
  delay(100);
  
  Serial.println(tokenIDStr);               // here you should replace "xxx..." with your Ubidots Token
  mySerial.println(tokenIDStr);
  delay(100);
  
  Serial.println(F("Host: things.ubidots.com"));
  mySerial.println(F("Host: things.ubidots.com"));
  delay(100);
  
  Serial.println();
  delay(100);
  
  Serial.println(var);
  mySerial.println(var);
  delay(100);
  
  Serial.println();
  delay(100);
  
  Serial.println((char)26);   // CONTROL+Z
  mySerial.println((char)26);
  delay(7000);
  
  Serial.println("");
  PrintSeconds();
  ShowSerialData();

  PrintSeconds();
  mySerial.println(F("Close connection to Ubidots: "));              // Close the connection
  sendATcommand("AT+CIPCLOSE","OK",2000);
}


void ShowSerialData()
{
  mySerial.println(F("Serial response"));
  while(Serial.available()!=0)
    mySerial.write(Serial.read());
}

void leerIR() {
  
  //PrintSeconds();
  //Serial.println(F("Leyendo IR"));
  if (irrecv.decode(&IR_resultados)) {

    irrecv.resume(); // Recibir el siguiente valor
    mySerial.print(F("IR valor: "));
    mySerial.println(IR_resultados.value);
    
    // Boton "MUTE" de mando SAMSUNG barra de sonido = 4294967295
    // Boton "CH-" del mando de Jorge, valor 16753245
    if ((IR_resultados.value == 4294967295) | (IR_resultados.value == 1434803654)) {
      switch(LEDSMode) {
        case 0:
          LEDSMode = 1;
          mySerial.println("Cambiando de modo 0 a 1");
          delay(1000);
          break;
        case 1:
          LEDSMode = 0;
          mySerial.println("Cambiando de modo 1 a 0");
          delay(1000);
          break;
        /* Otros modos para implementar en el futuro
        case 2: 
          LEDSMode = 3;
          break;
        case 3:
          LEDSMode = 0;
          break;
        */
      }
     PrintSeconds();
     mySerial.print(F("LEDS mode: "));
     mySerial.println(LEDSMode);
     printMode(LEDSMode);
    }
  }
}

void encenderLedsSonido() {
  
   if (currentRead == READS) {
    int promedio = 0;
    for (int i = 0; i < READS;i++) {
      promedio = promedio + values[i];
    }
    
    promedio = promedio / READS;
    
    if (promedio > 360) {
      
      timer_old_sonido = millis(); //Reseteamos el temporizador
      
      mySerial.print("Promedio: ");
      mySerial.println(promedio);
      setLedColor(promedio);

    } else {
    
      if ((millis() - timer_old_sonido)>(tiempo_maximo_sonido*1000)) {
        analogWrite(REDPIN, 0);
        analogWrite(BLUEPIN, 0);
        analogWrite(GREENPIN, 0);
        timer_old_sonido = millis();
      }
    }
    currentRead = 0;
  }
  
  values[currentRead] = analogRead(MIC_PIN);                        // Raw reading from mic 
  mySerial.print(F("Read "));
  mySerial.print(currentRead);
  mySerial.print(F(":"));
  mySerial.println(values[currentRead]);
  currentRead++;

}

void encenderLedsTemp() {

  if ((millis() - timer_old_LEDS)>(tiempo_maximo_LEDS*1000)) {
    timer_old_LEDS = millis();
    // Leer temperatura
    int t;
    float temp = dht.readTemperature();
    if (isnan(temp)) {
      mySerial.println(F("Failed to read from DHT sensor!"));
      return;
    }

    t = (int) temp; // Hacemos cast de temp para convertirlo de float a int
    if (t < 11) t = 10;
    if (t > 24) t = 25;
    /*
    PrintSeconds();
    mySerial.print("LEDs temp: ");
    mySerial.println(t);
    */
    
    switch (t) {
      case 10:
        analogWrite(REDPIN, 0);
        analogWrite(GREENPIN, 0);
        analogWrite(BLUEPIN, 255);
        break;
      case 11:
        analogWrite(REDPIN, 0);
        analogWrite(GREENPIN, 128);
        analogWrite(BLUEPIN, 255);
        break;
      case 12:
        analogWrite(REDPIN, 51);
        analogWrite(GREENPIN, 153);
        analogWrite(BLUEPIN, 255);
        break;
      case 13:
        analogWrite(REDPIN, 102);
        analogWrite(GREENPIN, 178);
        analogWrite(BLUEPIN, 255);
        break;
      case 14:
        analogWrite(REDPIN, 153);
        analogWrite(BLUEPIN, 255);
        analogWrite(GREENPIN, 204);
        break;
      case 15:
        analogWrite(REDPIN, 204);
        analogWrite(GREENPIN, 229);
        analogWrite(BLUEPIN, 255);
        break;
      case 16:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 204);
        break;
      case 17:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 153);
        break;
      case 18:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 102);
        break;
      case 19:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 51);
        break;
      case 20:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 255);
        analogWrite(BLUEPIN, 0);
        break;
      case 21:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 204);
        analogWrite(BLUEPIN, 204);
        break;
      case 22:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 153);
        analogWrite(BLUEPIN, 153);
        break;
      case 23:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 102);
        analogWrite(BLUEPIN, 102);
        break;
      case 24:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 51);
        analogWrite(BLUEPIN, 51);
        break;
      case 25:
        analogWrite(REDPIN, 255);
        analogWrite(GREENPIN, 0);
        analogWrite(BLUEPIN, 0);
        break;
    }
  }
}

void apagarLedsTemp() {

    analogWrite(REDPIN, 0);
    analogWrite(BLUEPIN, 0);
    analogWrite(GREENPIN, 0);
    
}

void leerSensores() {

    //Leemos temporizador y hacemos lecturas de sensores si se cumple el tiempo
    
      lcd.setCursor(7,1);
      lcd.print(((tiempo_maximo_ubidots*1000)-(millis()-timer_old_ubidots))/1000);
      lcd.print(F(" "));
      lcd.setCursor(11,1);
      //lcd.print(F("S:"));
      //lcd.print(((tiempo_maximo_sensors*1000)-(millis()-timer_old_sensors))/1000);
      //lcd.print(F("  "));
    
    // Leemos sensores y mostramos en LCS si se cumple el tiempo
    if ((millis() - timer_old_sensors)>(tiempo_maximo_sensors*1000)) {
        
        // Leer temperatura
        float temp = dht.readTemperature();
  
        // Leer humedad
        float hum = dht.readHumidity();
  
        // Check if any reads failed and exit early (to try again).
        if (isnan(temp) || isnan(hum)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
        }
        
        printLCD(temp,hum);
        
        // Reestablecemos temporizador
        timer_old_sensors = millis();
    }
    
    // Enviamos datos si se cumple el tiempo
    if ((millis() - timer_old_ubidots)>(tiempo_maximo_ubidots*1000)) {
        
        // Leer temperatura
        float temp = dht.readTemperature();
  
        // Leer humedad
        float hum = dht.readHumidity();
  
        // Check if any reads failed and exit early (to try again).
        if (isnan(temp) || isnan(hum)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
        }
    
          
    printLCD(temp,hum);
    lcd.setCursor(7,1);
    lcd.print(F("AHORA"));
  
    PrintSeconds();
    Serial.print(F("Temp: "));
    Serial.println(temp);
  
    PrintSeconds();
    Serial.print(F("Humedad: "));
    Serial.println(hum);
  
    //PrintSeconds();
    //Serial.print(F("Luz: "));
    //Serial.println(luz);
  
    // Encendemos fona
    TurnOnModem();
    delay(3000);
  
    // Enviamos temperatura a Ubidots
    PrintSeconds();
    Serial.println(F("************ Enviando temperatura a Ubidots"));
    Send2ubidots(String(temp),String(F("54dcde5876254240b3af12a8")),String(token));
  
    // Enviamos % voltaje de bateria Li-Po
    // PrintSeconds();
    // Serial.println(F("************ Enviando % voltaje bateria LiPo a Ubidots"));
    // Send2ubidots(String(vbat),String(F("54e70de57625422f2f2313da")),String(token));
  
    // Enviamos humedad a Ubidots
    PrintSeconds();
    Serial.println(F("************ Enviando humedad a Ubidots"));
    Send2ubidots(String(hum),String(F("54e72f8d76254260d428020e")),String(token));
  
    // Enviamos luz a Ubidots
    //PrintSeconds();
    //Serial.println(F("************ Enviando luz a Ubidots"));
    //Send2ubidots(String(luz),String(F("54e772ac7625423c39a651d0")),String(token));
  
    // Apagamos fona
    TurnOffModem();
    
    // Reestablecemos temporizador
    timer_old_ubidots = millis();
    }
}

void printMode(int mode) {
  lcd.setCursor(13,0); //Caracter 13, linea 0
  lcd.print(F("M:"));
  lcd.print(mode);
}

