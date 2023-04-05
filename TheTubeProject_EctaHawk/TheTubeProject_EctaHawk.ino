#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#include "mbed.h"
#include "FanManager.h"
#include "Potentiometer.h"
#include "HCSR04.h"
//#include "NRF52_MBED_TimerInterrupt.h" // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
//#include "NRF52_MBED_ISR_Timer.h" // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <SimpleTimer.h>
#include <ArduinoBLE.h>

/*********************************************************
           BLE
**********************************************************/
BLEService fanService("1101");
BLEIntCharacteristic fanSpeedCharacteristic("2101", BLERead | BLEWrite);

/*********************************************************
           INSTANCIATION DES PERIPHERIQUES
**********************************************************/
float _fan2Setpoint=0.5;
float _Quiet=LOW;
int i=0;
bool start = true;
bool ramp = false;
int setpointRPM;
int realSetpoint;
String command;

float _mainFanSpeed;
float _secondaryFanSpeed;
float plotHeightCm;
float plotHeightPercent;
float _externalSetpoint;

long int tExecSpeedTask;
long int tExecPosTask;
long int tExecUserTask;
long int tExecMonTask;
long int tTemp;

//BLE
BLEDevice central;

#define MainFanEnablPin D2
#define MainFanPWMPin D3
#define MainFanHallPin D7
#define MainFanCurrentPin A2

#define SecondaryFanEnablPin D6
#define SecondaryFanPWMPin D5
#define SecondaryFanHallPin D9
#define SecondaryFanCurrentPin A3

#define PotentiometerPin A0

#define HS_TrigPin D4
#define HS_EchoPin D8

FanManager mainFan(MainFanPWMPin,MainFanHallPin,MainFanEnablPin,MainFanCurrentPin);

FanManager secondaryFan(SecondaryFanPWMPin,SecondaryFanHallPin,SecondaryFanEnablPin,SecondaryFanCurrentPin);

Potentiometer consigneExterne(PotentiometerPin);

HCSR04 heightSensor(HS_TrigPin, HS_EchoPin);

/*********************************************************
          GESTION DES INTERRUPTIONS (HARDWARE)
**********************************************************/
/*
Afin de pouvoir évaluer la vitesse des ventilateurs, il est
nécéssaire de comter les impulsions des capteurs Hall des
ventilateurs.
La fonction de callback ne peut pas être une méthode de classe,
a moins qu'elle soit statique. Comme on a deux ventilateur,
On a choisi de créer des fonction de callback dans le code 
principal, qui vont elle-même apeler en cascade les méthode
des deux instances de ventilateurs.
*/

void mainFan_incRpmCounter()
{
  mainFan.incRpmCounter();
}

void SecondaryFan_incRpmCounter()
{
  secondaryFan.incRpmCounter();
}

void setupFanInterrupts()
{
  pinMode(mainFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(mainFan.getHallPinNumber()), mainFan_incRpmCounter, CHANGE);

  pinMode(secondaryFan.getHallPinNumber(), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(secondaryFan.getHallPinNumber()), SecondaryFan_incRpmCounter, CHANGE);
}

void setupBLE()
{ 
  if (!BLE.begin()) 
  {
    Serial.println("Erreur lors de l'initialisation du BLE !");
    while (1);
  }

  BLE.setLocalName("Ventilateur");
  BLE.setAdvertisedService(fanService);

  fanService.addCharacteristic(fanSpeedCharacteristic);

  BLE.addService(fanService);

  fanSpeedCharacteristic.writeValue(0); // Initialisation de la valeur de la vitesse du ventilateur

  BLE.advertise();
  Serial.println("Prêt à accepter les connexions");
}

/*********************************************************
               GESTION DES TACHES ET TIMERS
**********************************************************/
/*
Le programme est séparé en plusieurs tâches.
Afin d'avoir un minimum de synchronisation dans l'ordre des opérations, 
on utilisera un timer hardware principal, qui sera chargé d'éxécuter
les différentes autres tâches critiques avec un système de diviseurs.
La tâche user_Ctrl, moins critique au niveau du timing, sest gérée par un 
timer software (moins précis, soft realtime)
La tâche monitoring est séparée du reste et appelée dans la
loop() arduino. Elle est rythmée par une pause, ce qui ne la rend pas temps réel.

Vous pouvez librement adapter la porposition ci-dessous, mais n'oubliez pas de
le justifier dans la rapport, ce qui me permettra de comprendre plus façilement
ce que vous avez fait.

Attention, l'utilisation de la communication serielle n'est pas anodine en terme
de consommation des ressources. Veillez à limiter au maximum les print dans les
tâches synchronisées et laissez dans la mesure du possible cette tâche à la
tâche de monitoring.

Chacune de ces tâches sera organisée en terme de séquencement.
On commence par récupérer les valeurs (Input), on calcul ensuite les sorties (Compute)
puis on affecte finnalement les sorties physiques ou de la tâche suivante (Ouput).

Le rôle de chacune des tâches est décrit ci-dessous:

speed_Ctrl_Task:
  - Récupère la valeur de la vitesse des ventilateur
  - Récupère la dernière consigne de vitesse
  - Calcule la nouvelle sortie/correction
  - Affecte la sortie sur le pwm

position_Ctrl_Task:
  - Récupère la valeur de position du capteur
  - Récupère la dernière consigne de position
  - Calcule la nouvelle sortie/correction
  - Affecte la sortie pour la boucle de vitesse

user_Ctrl_Task:
  - Récupère la dernière consigne selon le mode
  - Contrôle si une action utilisateur est demandée
  - Exécute l'action et calcule la nouvelle consigne le cas échéant
  - Affecte la sortie pour la boucle de postion

monitoring_Task
  - fait une copie des dernières valeurs à remonter
  - Met en forme pour l'affichage
  - Envoie la tramme sur le sortie serielle

NB: D'autre tâches peuvent apparaître au cours du projet, il s'agit ici de la base de l'application.
*/

void speed_Ctrl_Task()
{
  tTemp = micros();
  _mainFanSpeed = mainFan.computeSpeedRPM();
  _secondaryFanSpeed = secondaryFan.computeSpeedRPM();
  //Compute
  if(!start)
  {
    mainFan.setSpeedProp(0);
    secondaryFan.setSpeedProp(0);
  }
  else
  {
    mainFan.setSpeedProp(float(_externalSetpoint/100));
    secondaryFan.setSpeedProp(_fan2Setpoint);
  }

  //Output
  mainFan.enableRotation(!_Quiet);
  secondaryFan.enableRotation(!_Quiet);
  tExecSpeedTask = micros() - tTemp;
}

void position_Ctrl_Task()
{
  tTemp = micros();
  //Inputs
  plotHeightPercent = heightSensor.measureDistance();
 
  //Compute
  // Calcul de la distance en cm en utilisant l'équation de droite
  plotHeightCm = heightSensor.heightInCm();
  //Ouput

  tExecPosTask = micros() - tTemp;
}

void user_Ctrl_Task()
{
  tTemp = micros();
  _externalSetpoint = consigneExterne.getValuePercent(); //TODO AGC -> Should be here, but it seem analog read into interrupt dont work...Use continous ADC Read?

  //Récupération de la commande
  if(Serial.available())
  {
    command = Serial.readStringUntil('\n');
         
    if(command.equals("start"))
    {
      start = true;
    }
    else if(command.equals("inc"))
    {
      i++;
    }
    else if(command.equals("dec"))
    {
      i--;
    }
    else if(command.equals("ramp"))
    {
      ramp = !ramp;
    }
    else if(command.equals("stop"))
    {
      start = false;
    }
    else if(command.equals("reset"))
    {
      NVIC_SystemReset();                         // Reset the microcontroller
    }
        else if(command.indexOf("Kp") == 0) //The command format must be Kp=xx.xx
    {
      //Kp=command.substring(3).toDouble();
    }
    else if(command.indexOf("Ki") == 0) //The command format must be Ki=xx.xx
    {
      //Ki=command.substring(3).toDouble();
    }
    else if(command.indexOf("Kd") == 0) //The command format must be Kd=xx.xx
    {
      //Kd=command.substring(3).toDouble();
    }
    else if(command.indexOf("Fan2") == 0) //The command format must be Fan2=xx.xx
    {
      _fan2Setpoint=command.substring(5).toDouble();
    }
     else if(command.equals("Quiet")) 
    {
      _Quiet=!_Quiet;
    }
    else
    {
      Serial.println("Invalid command");
    }
  }

  //TODO
  tExecUserTask = micros() - tTemp;
}

//#define MONITOR 
//#define PLOT_TIMINGS

//monitoring_Task
void monitoring_Task()
{
  tTemp = micros();
    
#ifdef MONITOR
  Serial.print ("cons_ext.:");
  Serial.print (_externalSetpoint, 1);
  Serial.print (",MainFanRpm:");
  Serial.print (_mainFanSpeed, 0);
  Serial.print (",SecondaryFanRpm:");
  Serial.print (_secondaryFanSpeed, 0);
  Serial.print (",setpointRPM:");
  Serial.print (setpointRPM, DEC);
  Serial.print (",realSetpoint:");
  Serial.print (realSetpoint, DEC);
  Serial.print (",HauteurCM:");
  Serial.print (plotHeightCm,1);
  Serial.print (",HauteurPercent:");
  Serial.print (plotHeightPercent,1);
  Serial.print ("\r\n");
#endif

#ifdef PLOT_TIMINGS
  Serial.println ("Application timings: ");
  Serial.println ("Speed Task: "+String(float(tExecSpeedTask)/1000)+" ms");
  Serial.println ("Pos Task: "+String(float(tExecPosTask)/1000)+" ms");
  Serial.println ("User Task: "+String(float(tExecUserTask)/1000)+" ms");
  Serial.println ("Monitoring Task: "+String(float(tExecMonTask)/1000)+" ms");
#endif

    tExecMonTask = micros() - tTemp;
}

void BLETask()
{
  if (central) {
    if (central.connected()) {
      if (fanSpeedCharacteristic.written()) {
        int newFanSpeed = fanSpeedCharacteristic.value();
        realSetpoint = mainFan.setSpeed(newFanSpeed);
        Serial.print("Nouvelle vitesse du ventilateur principal : ");
        Serial.println(newFanSpeed);
      }
    }
  }
  else
  {
    central = BLE.central();
  }
}

#define SPEED_TASK_PERIOD_MS 43
#define POS_TASK_PERIOD_MS 11
#define MONITORING_TASK_PERIOD_MS 100

//Calcul de la période de la tâche
#define TIMER_TASK_US (SPEED_TASK_PERIOD_MS+POS_TASK_PERIOD_MS)*1000 //Périodes des tâches SPEED et POSITION convertit en microsecondes.

// the soft timer object
SimpleTimer timerTask;
SimpleTimer timerMonitor;
int counterHard=0;
int counterSoft=0;

void HandlerTickTask()
{
  //No print inside these
  speed_Ctrl_Task();
  position_Ctrl_Task();
}

void HandlerTickMonitor() {
  //user_Ctrl_Task();

  monitoring_Task();
}

/*********************************************************
                      APPLICATION
**********************************************************/
#define NBR_DIG 2

void setup()
{
  analogReadResolution(12);

  interrupts(); 

  setupFanInterrupts();

  timerTask.setInterval(TIMER_TASK_US, HandlerTickTask); 
  timerMonitor.setInterval(MONITORING_TASK_PERIOD_MS*1000, HandlerTickMonitor);
  
  Serial.begin(115200);

  setupBLE();
}

void loop() 
{ 
  //Serial.println("Temps d'execution speed: "+String(tExecSpeedTask));
  //Serial.println("Temps d'execution position: "+String(tExecPosTask)); 

  timerTask.run();
  timerMonitor.run();
}
