#include <NewPing.h>
#include <Servo.h>
/*Objeto de la clase servo*/
Servo barridoUltrasonico;

/*Variables para la lectura de los sensores IR*/
const int trenPulsos = 16;
const int sensorIzquierda = A5;
const int sensorDerecha = A0;
int valorSensorDerecha=0;
int valorSensorIzquierda=0;
int IrAmbiente=0;             // variable para guardar la luz IR en el ambiente
int IrReflejo=0;              // Comparacion entra ambiente e IR de reflejo
int distanciaIR=0;            // Variable para almacenar el valor analogico obtenido luego de la lecura
int valoresIR[10];            // Vector para guardar lecturas

/*Motores*/
const int motorIzquierdaAvanza = 8;
const int motorIzquierdaRetrocede = 7;
const int velocidadMotorIzquierda = 5;

const int motorDerechaAvanza = 10;
const int motorDerechaRetrocede = 9;
const int velocidadMotorDerecha = 6;

/*Variables para el escaneo Ultrasonico*/
/*int lecturas[10];
 int posiciones[10] = {30,45,60,75,90,105,120,135,150};*/
//const int posDerecha = 175;     //Limite superior hasta el que se movera el servo para evitar fatiga
//const int posIzquierda = 5;      //Limite inferior hasta el que se movera el servo para evitar fatiga
const int trig = 12;             //Pin para activar la emision de ultrasonido
const int echo = 13;             //Pin de lectura de tiempo de duracion del eco
const int distanciaMaxima = 200; //Distancia maxima de medicion en centimetros
int pos = 0;                     //Posicion en grados para mover al Servo
int distanciaCentimetros=0;      
int lecturaMayor=0;
int distanciaCentral=0;
int distanciaIzquierda=0;
int distanciaDerecha=0;
int posicionVector=0;
NewPing sonar(trig, echo, distanciaMaxima);

void setup()
{
  //Serial.begin(115200);
  barridoUltrasonico.attach(3); 
  //Pin que se utilizara para mover el sonar
  barridoUltrasonico.write(90);                //Colocar el sonar en la posicion central
  
  pinMode(motorIzquierdaAvanza, OUTPUT);
  pinMode(motorIzquierdaRetrocede, OUTPUT);
  pinMode(velocidadMotorIzquierda, OUTPUT);

  pinMode(motorDerechaAvanza, OUTPUT);
  pinMode(motorDerechaRetrocede, OUTPUT);
  pinMode(velocidadMotorDerecha, OUTPUT); 

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
//Test de calibracion de giro
  //giroDerecha(255);
  //giroIzquierda(245);
  //giroIzquierda(440);
}


int medicion()
{
  delay(50);                        // Esperar 50 ms entre pings (unos 20 pings / seg). 29ms debe ser el plazo más breve entre pings.
  unsigned int uS = sonar.ping();   // Enviar ping, obtener el tiempo de ping en microsegundos (uS).
  unsigned int resultado=0;
  resultado=(uS / US_ROUNDTRIP_CM); // Convertir el tiempo de un ping a la distancia en cm.
  return(resultado);
}

/*Funcion que crea el tren de pulsos para hacer funcionar los sensores
 Y calcular luego el promedio entre las lecturas
 ENTRADAS:
 lecturas: Cantidad de veces que se leeran los sensores IR
 pinDigital: El pin que se utilizara para el tren de pulsos
 pinAnalogo: El pin en el cual ira la salida del sensor
 */
int lecturaInfrarroja(int lecturas, int pinDigital, int pinAnalogo){
  distanciaIR=0;
  for(int x=0;x<lecturas;x++){
    digitalWrite(pinDigital,LOW);     //Apaga los emisores IR para leer la IR ambiente
    delay(1);                             // Retardo minimo necesario para tomar lecturas
    IrAmbiente = analogRead(pinAnalogo);  // Guardar la luz IR ambiental
    digitalWrite(pinDigital,HIGH);        //Enciende los emisores IR para leer la luz que reflejan los objetos
    delay(1);                        
    IrReflejo = analogRead(pinAnalogo);   // Guardar IR que refleja de los obstaculos
    valoresIR[x] = IrAmbiente-IrReflejo;  // Calcula los cambios IR y luego los guarda para poder promediarlos
  }

  for(int x=0;x<lecturas;x++){            // Se calcula el promedio
    distanciaIR+=valoresIR[x];
  }
  return(distanciaIR/lecturas);           // Valor que tomara la variable que llame a la funcion
} //Fin funcion de lectura IR

/*Funciones para el manejo del vehiculo*/
/*Funcion para controlar la velocidad de los motores utilizando PWM
 motorA: motor izquierdo
 motorB: motor derecho
 */
void velocidadMotores(int motorA, int motorB)
{
  analogWrite(velocidadMotorIzquierda, motorA);
  analogWrite(velocidadMotorDerecha, motorB);
}

void apagarMotores(int opcion)//1:Proteccion para avanzar 2: Proteccion para Retroceder 3: parar
{
  switch(opcion)
  {
  case 1: 
    digitalWrite(motorIzquierdaRetrocede, LOW);
    digitalWrite(motorDerechaRetrocede, LOW);
    break;

  case 2: 
    digitalWrite(motorIzquierdaAvanza, LOW);
    digitalWrite(motorDerechaAvanza, LOW);
    break;

  case 3: 
    digitalWrite(motorIzquierdaAvanza, LOW);
    digitalWrite(motorDerechaAvanza, LOW);
    digitalWrite(motorIzquierdaRetrocede, LOW);
    digitalWrite(motorDerechaRetrocede, LOW);
    break;
  }

}

void Avanzar()
{
  apagarMotores(1);
  velocidadMotores(90,90);
  digitalWrite(motorIzquierdaAvanza, HIGH);
  digitalWrite(motorDerechaAvanza, HIGH);
}

void Retroceder()
{
  apagarMotores(2);
  velocidadMotores(150,150);
  digitalWrite(motorIzquierdaRetrocede, HIGH);
  digitalWrite(motorDerechaRetrocede, HIGH);
}

void voltearIzquierda(int tiempo)
{
  apagarMotores(3);
  Avanzar();
  velocidadMotores(0,150);
  delay(tiempo);
  apagarMotores(3);
}

void voltearDerecha(int tiempo)
{
  apagarMotores(3);
  Avanzar();
  velocidadMotores(150,0);
  delay(tiempo);
  apagarMotores(3);
}

void giroIzquierda(int tiempo)
{
  apagarMotores(3);
  delay(100);
  velocidadMotores(150,150);
  digitalWrite(motorIzquierdaRetrocede, HIGH);
  digitalWrite(motorDerechaAvanza, HIGH);
  delay(tiempo);
  apagarMotores(3);  
}

void giroDerecha(int tiempo)
{
  apagarMotores(3);
  delay(100);
  velocidadMotores(150,150);
  digitalWrite(motorDerechaRetrocede, HIGH);
  digitalWrite(motorIzquierdaAvanza, HIGH);
  delay(tiempo);
  apagarMotores(3); 
}

void compararDistancias()
{
  if(distanciaIzquierda>distanciaDerecha)
  {
    giroIzquierda(245);
  }
  else if(distanciaDerecha>distanciaIzquierda)
  {
    giroDerecha(255);
  }
  else if(distanciaDerecha<=5&&distanciaIzquierda<=5)
  {
    apagarMotores(3);
    Retroceder();
    delay(200);
    giroIzquierda(440);
  }

}

void obstaculoIndetectable(int izquierda,int derecha)
{

}

void loop()
{
  valorSensorDerecha = 0;
  valorSensorIzquierda=0;
  valorSensorDerecha = lecturaInfrarroja(5,2,0);   //Asignar los valores de las lecturas IR    
  valorSensorIzquierda = lecturaInfrarroja(5,2,5); 
  int valorDiferencialDerecha = valorSensorDerecha-valorSensorIzquierda;
  int valorDiferencialIzquierda = valorSensorIzquierda-valorSensorDerecha;
  if(valorSensorIzquierda >=10 && valorSensorDerecha >=10 && valorDiferencialDerecha >= 3)
  {
    voltearIzquierda(200);
    Avanzar();
  }
  else
    if(valorSensorIzquierda >=10 && valorSensorDerecha >=10 && valorDiferencialIzquierda >= 3)
    {
      voltearDerecha(200);
      Avanzar();
    }

  if(valorSensorDerecha>=20 && valorSensorDerecha<=50 && valorSensorIzquierda>=20 && valorSensorIzquierda<=50)
  {
    apagarMotores(3);
    Retroceder();
    delay(200);
    giroIzquierda(440);
  }
  else
    if(valorSensorDerecha>=50||valorSensorIzquierda>=50)
    {
      apagarMotores(3);
      Retroceder();
      delay(200);
      giroIzquierda(440);
    }

  distanciaCentral = medicion();
  if(distanciaCentral > 15)
  {
    Avanzar();
  }
  else
  {
    apagarMotores(3);
    barridoUltrasonico.write(173);
    delay(300);
    distanciaIzquierda = medicion();
    delay(300);
    barridoUltrasonico.write(7);
    delay(600);
    distanciaDerecha = medicion();
    delay(300);
    barridoUltrasonico.write(90);
    delay(100);
    compararDistancias();
  }
}







