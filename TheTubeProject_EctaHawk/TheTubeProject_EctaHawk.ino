#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#include "mbed.h"
#include "FanManager.h"
#include "Potentiometer.h"
#include "HCSR04.h"
#include "NRF52_MBED_TimerInterrupt.h" // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "NRF52_MBED_ISR_Timer.h" // To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include <SimpleTimer.h>
#include <ArduinoBLE.h>

/*********************************************************
           BLE
**********************************************************/
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

/*********************************************************
           INSTANCIATION DES PERIPHERIQUES
**********************************************************/
int i=0;
bool start = true;
bool ramp = false;
int setpointRPM;
int realSetpoint;
String command;

float _mainFanSpeed;
float _secondaryFanSpeed;
float _plotHeight;
float _externalSetpoint;

long int tExecSpeedTask;
long int tExecPosTask;
long int tExecUserTask;
long int tExecMonTask;
long int tTemp;

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
  // set advertised local name and service UUID:
  BLE.setLocalName("TheTube");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
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
  //TODO
  tExecSpeedTask = micros() - tTemp;
}

void position_Ctrl_Task()
{
  tTemp = micros();
  _plotHeight = heightSensor.measureDistance();
  //TODO
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
    else
    {
      Serial.println("Invalid command");
    }
  }

  //TODO
  tExecUserTask = micros() - tTemp;
}

#define SPEED_TASK_PERIOD_MS 200
#define POS_TASK_MUL  2 //Give position task time = POS_TASK_MUL*SPEED_TASK_PERIOD
#define USER_TASK_MUL  4 //Give user task time = USER_TASK_MUL*SPEED_TASK_PERIOD
#define MON_TASK_MUL  8 //Give monitoring task time = MON_TASK_MUL*SPEED_TASK_PERIOD

#define TIMER_INTERVAL_US        SPEED_TASK_PERIOD_MS*1000      // 1s = 1 000 000us

// Init NRF52 hard timer NRF_TIMER3
NRF52_MBED_Timer ITimer(NRF_TIMER_4);
// the soft timer object
SimpleTimer timer;
int counter=0;

void HandlerTickTaskHard()
{
  // Call the different Tasks here inside ISR
  // No Serial.print() can be used

  speed_Ctrl_Task();
  //BLE_task();
  
  if(counter % POS_TASK_MUL)
  {
    position_Ctrl_Task();
  }  
  counter++;
}

void HandlerTickTaskSoft() {
    user_Ctrl_Task();
}

/*********************************************************
                      APPLICATION
**********************************************************/
#define MONITOR 
#define NBR_DIG 2

void setup()
{
  analogReadResolution(12);

  interrupts(); 

  setupFanInterrupts();

  ITimer.attachInterruptInterval(TIMER_INTERVAL_US, HandlerTickTaskHard);    

  timer.setInterval(SPEED_TASK_PERIOD_MS*USER_TASK_MUL, HandlerTickTaskSoft);
  
  Serial.begin(115200);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  setupBLE();
}

void loop() //monitoring_Task
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  mainFan.enableRotation(start);
  secondaryFan.enableRotation(false);

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      //tTemp = micros();
      //timer.run();

      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      mainFan.setSpeed(switchCharacteristic.value());
    }
  }
  else
  {
    tTemp = micros();
    timer.run();
  
    mainFan.setSpeed(0);
  }  
}