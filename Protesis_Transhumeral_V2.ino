//Daniel Escalada

//Reconocedor voz
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

//LCD
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Servo
#include <VarSpeedServo.h>

//SCL -> A5 y SDA -> A4
//Objeto lcd -> "lcd"
LiquidCrystal_I2C lcd(0x27,16,2);   

//Objeto voz -> "myVR" (TX,RX) 
VR myVR(2,3);    

//Vector de comandos guardados (loaded)
uint8_t records[7];
//Vector de comandos completo 
uint8_t buf[64];

//Asignacion de posicion de los comandos en el vector records[7]
#define codo       (0)  //q1
#define munyeca    (1)  //q3
#define antebrazo  (2)  //q2
#define pinza      (3) 
#define abrir      (4)
#define cerrar     (5) 
#define para       (6)

//Pines motores
#define q1 5   
#define q2 6
#define q3 7

#define en 11  // Enable Motor DC
#define i1 10  // Control_Pin_1 Motor DC
#define i2 9   // Control_Pin_2 Motor DC

//Objetos servo

VarSpeedServo servo_q1, servo_q2, servo_q3;

//Variables

float Sensibilidad = 0.185;  //Sensibilidad sensor

float corrienteBase;          

enum estadosMov {ABRIR,CERRAR,PARA};
enum estadosBrazo {CODO,MUNYECA,ANTEBRAZO,PINZA};

estadosMov estadoMov = PARA;   //Se inicializa en parado
estadosBrazo estadoBrazo = CODO;

//Funcion muestreo de corriente

float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(A3) * (5.0 / 1023.0);        //Lectura del sensor (A3)
    corriente=corriente+(voltajeSensor-2.5)/Sensibilidad;   //Ecuación  para obtener la corriente
  }
  corriente=corriente/n_muestras;
  return(corriente);
}
//Funcion parada 

void parada_Pinza(){

if (get_corriente(500) <= (corrienteBase - 0.3) | get_corriente(500) >= (corrienteBase + 0.3)){  //Si se alcanza un límite
 
  digitalWrite(en, LOW);  //Parar motor
  Serial.println("  PARADA"); 
  estadoMov = PARA; ???
}
}
//Funciones internas del reconocedor de voz (no tocar)
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

//////////////////////////////////////////////////////////////////////

void setup()
{
  //LCD
  //Inicializar el LCD
  lcd.init();
  
  //Encender la luz de fondo.
  lcd.backlight();

  //Voz
  //Inicializar
  myVR.begin(9600);
  
  Serial.begin(115200);
  Serial.println("Elechouse Voice Recognition V3 Module\r\n");
  

    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  //Mensajes de carga de comandos
  if(myVR.load((uint8_t)codo) >= 0){
    Serial.println("codo loaded");
  }
  
  if(myVR.load((uint8_t)munyeca) >= 0){
    Serial.println("munyeca loaded");
  }

  if(myVR.load((uint8_t)antebrazo) >= 0){
    Serial.println("antebrazo loaded");
  }

  if(myVR.load((uint8_t)pinza) >= 0){
    Serial.println("pinza loaded");
  }

  if(myVR.load((uint8_t)abrir) >= 0){
    Serial.println("abrir loaded");
  }

  if(myVR.load((uint8_t)cerrar) >= 0){
    Serial.println("cerrar loaded");
  }

  if(myVR.load((uint8_t)para) >= 0){
    Serial.println("para loaded");
  }

  //Servo
  servo_q1.attach(q1);
  servo_q2.attach(q2);
  servo_q3.attach(q3);

  pinMode(en, OUTPUT);    
  pinMode(i1, OUTPUT);
  pinMode(i2, OUTPUT); 

  corrienteBase = get_corriente(1000);
}

//////////////////////////////////////////////////////////////////////

void loop()
{

  //Funcion parada pinza
  parada_Pinza();

  //Switch principal de comandos
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      //CODO
      case codo:
        //Mover codo
        estadoBrazo = CODO;
        break;
      //MUÑECA
      case munyeca:
        //Mover muñeca
        estadoBrazo = MUNYECA;
        break;
      //ANTEBRAZO
      case antebrazo:
        //Mover antebrazo
        estadoBrazo = ANTEBRAZO;
        break;
      //PINZA
      case pinza:
        //Mover pinza
        estadoBrazo = PINZA;
        break;
      //ABRIR
      case abrir:
        estadoMov = ABRIR;
        //CASOS
        //Pinza
        if (estadoBrazo == PINZA){
          analogWrite(en, 200); 
          digitalWrite(i1, LOW); 
          digitalWrite(i2, HIGH);
          break;
        }
        //Codo
        else if (estadoBrazo == CODO){
          servo_q1.write(140,15);
          break;
        }
        //Muñeca
        else if (estadoBrazo == MUNYECA){
          servo_q3.write(30,15);
          break;
        }
        //Antebrazo
        else if (estadoBrazo == ANTEBRAZO){
          servo_q2.write(0,15);
          break;
        }
        //Ningun estado brazo seleccionado
        else 
        break;
      //CERRAR
      case cerrar:
        estadoMov = CERRAR;
        //CASOS
        //Pinza
        if (estadoBrazo == PINZA){
        analogWrite(en, 200); 
        digitalWrite(i1, HIGH); 
        digitalWrite(i2, LOW);
        break;
         }
        //Codo
        else if (estadoBrazo == CODO){
          servo_q1.write(180,15);
          break;
        }
        //Muñeca
        else if (estadoBrazo == MUNYECA){
          servo_q3.write(180,15);
          break;
        }
        //Antebrazo
        else if (estadoBrazo == ANTEBRAZO){
          servo_q2.write(160,15);
          break;
        }
        //Ningun estado brazo seleccionado
        else 
        break;
      //PARA
      case para:
        estadoMov = PARA;
        //Parar montor pinza
        digitalWrite(en, LOW);  
        //Parar servos
        servo_q1.stop();
        servo_q2.stop();
        servo_q3.stop();
        break;
      //DEFAULT
      default:
        break;
    }

  //Voz reconocida
  printVR(buf);

  //Actulizacion lcd
  lcd.clear();
  //Primera linea
  switch (estadoBrazo){
    case CODO:
      lcd.println("CODO");
      break;
    case MUNYECA:
      lcd.println("MUNYECA");
      break;
    case ANTEBRAZO:
      lcd.println("ANTEBRAZO");
      break;
    case PINZA:
      lcd.println("PINZA");
      break;
    default:
      lcd.println("NADA");
      break;
    }
  //Segunda linea
  lcd.setCursor(0, 1);
  switch (estadoMov){
    case ABRIR:
      lcd.println("ABRIR");
      break;
    case CERRAR:
      lcd.println("CERRAR");
      break;
    case PARA:
      lcd.println("PARA");
      break;
    default:
      lcd.println("NADA");
      break;
    }

  }
}