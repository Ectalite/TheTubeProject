#if !( ARDUINO_ARCH_NRF52840 && TARGET_NAME == ARDUINO_NANO33BLE )
  #error This code is designed to run on nRF52-based Nano-33-BLE boards using mbed-RTOS platform! Please check your Tools->Board setting.
#endif

#include "mbed.h"
#include "FanManager.h"
#include "Potentiometer.h"
#include "HCSR04.h"
#include <SimpleTimer.h>

/*********************************************************
           INSTANCIATION DES PERIPHERIQUES
**********************************************************/
float _fan2Setpoint=0.5;
float _Quiet=LOW;
int i=0;
bool start = false;
bool ramp = false;
bool contest = false;
bool pot = false;
bool monitor = true;
bool regul_vitesse = true;
int setpointRPM;
String command;

float _mainFanSpeed;
float _secondaryFanSpeed;
float plotHeightCm;
float plotHeightPercent;
float HeightPercent;
float _externalSetpoint;
float _lastTrajSetpoint = 0;

long int tExecSpeedTask;
long int tExecPosTask;
long int tExecUserTask;
long int tExecMonTask;
long int tTemp;

//Constante pour la balle
const double Ku = 50;
const double Tu = 1.98;

//Ziegler Nichols
double Kp = 0.45*Ku;
double Ki = (0.54*Ku)/Tu;
double Kd = 0;

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

#define MONITOR
//#define BALLE
//Debug

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

int integral_speed = 0;

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
    if(contest)
    {
      mainFan.setSpeedProp(0);
      secondaryFan.setSpeedProp(0);
    }
    else if(pot)
    {
      mainFan.setSpeedProp(float(_externalSetpoint/100));
      secondaryFan.setSpeedProp(_fan2Setpoint);
    }
    else
    {
      if(regul_vitesse)
      {
        //Serial.print("Ping");
        if(setpointRPM < 3500)
        {
          setpointRPM = 3500;
        }
        int erreur = setpointRPM - _mainFanSpeed;
        integral_speed += erreur;
        int vitesse = 7 * erreur + 0.25 * integral_speed;
        mainFan.setSpeed(vitesse);
      }
      else
      {
        mainFan.setSpeed(setpointRPM);
      }
      secondaryFan.setSpeedProp(_fan2Setpoint);
    }
  }

  //Output
  mainFan.enableRotation(!_Quiet);
  secondaryFan.enableRotation(!_Quiet);
  tExecSpeedTask = micros() - tTemp;
}

float integral_pos = 0;
float erreur_pos = 0;

void position_Ctrl_Task()
{
  tTemp = micros();
  //Inputs
  HeightPercent = heightSensor.measureDistance();
  plotHeightPercent = HeightPercent;
  // Calcul de la distance en cm en utilisant l'équation de droite
#ifdef BALLE  
  plotHeightCm = heightSensor.heightInCm(HeightPercent);
#else
  plotHeightCm = (heightSensor.heightInCm(HeightPercent)-2.3);
#endif
  //Régulation
  erreur_pos = _lastTrajSetpoint - plotHeightCm;
  integral_pos += erreur_pos;
  
  //Ouput
  setpointRPM = Kp * erreur_pos + Ki * integral_pos;
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
      _lastTrajSetpoint = heightSensor.heightInCm(HeightPercent);
      integral_pos = 0;      
      Serial.println(start ? "Démarré" : "Arrêté");
    }
    else if(command.equals("inc"))
    {
      i++;
      Serial.println("i = "+ String(i));
    }
    else if(command.equals("dec"))
    {
      i--;
      Serial.println("i = "+ String(i));
    }
    else if(command.equals("ramp"))
    {
      ramp = !ramp;
      String state = ramp ? "activé" : "desactivé";
      Serial.println("Commande rampe " + state);
    }
    else if(command.equals("monitor"))
    {
      monitor = !monitor;
      String state = monitor ? "activé" : "desactivé";
      Serial.println("Commande monitor " + state);
    }
    else if(command.equals("potentiometre"))
    {
      pot = !pot;
      String state = pot ? "activé" : "desactivé";
      Serial.println("Commande potentiomètre " + state);
    }
    else if(command.equals("stop"))
    {
      start = false;
    }
    else if(command.equals("reset"))
    {
      NVIC_SystemReset();                         // Reset the microcontroller
    }
    else if(command.equals("regulation"))
    {
      regul_vitesse = !regul_vitesse;
      String state = regul_vitesse ? "activé" : "desactivé";
      Serial.println("Régulation de vitesse " + state);                    
    }
    else if(command.indexOf("Kp") == 0) //The command format must be Kp=xx.xx
    {
      Kp=command.substring(3).toDouble();
      Serial.println("Set Kp: " + String(Kp)); 
    }
    else if(command.indexOf("Ki") == 0) //The command format must be Ki=xx.xx
    {
      Ki=command.substring(3).toDouble();
      Serial.println("Set Ki: " + String(Ki));
    }
    else if(command.indexOf("Kd") == 0) //The command format must be Kd=xx.xx
    {
      Kd=command.substring(3).toDouble();
      Serial.println("Set Kd: " + String(Kd));
    }
    else if(command.indexOf("fan1") == 0) //The command format must be fan1=xx in rpm
    {
      setpointRPM=command.substring(5).toInt();
      Serial.println("Fan1 setspeed " + String(setpointRPM) + " RPM");
    }
    else if(command.indexOf("fan2") == 0) //The command format must be fan2=xx.xx
    {
      _fan2Setpoint=command.substring(5).toDouble();
      //Serial.println("Fan2 setspeed " + String(_fan2Setpoint));
    }
    else if(command.equals("contest"))
    {
      contest = true;
      start = false;
      _Quiet = LOW;
      _lastTrajSetpoint = 0;
    }
    else if(command.indexOf("setpoint") == 0) //The command format must be traj=<time>;<setpoint>
    {
      _lastTrajSetpoint = command.substring(9).toDouble();
      Serial.println("Setpoint: " + String(_lastTrajSetpoint));
    }
    else if(command.indexOf("traj") == 0) //The command format must be traj=<time>;<setpoint>
    {
      _lastTrajSetpoint = command.substring(21).toDouble();
      Serial.println(command.substring(5,command.length()-1) + ";" + String(plotHeightPercent));
    }
     else if(command.equals("quiet")) 
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

//#define PLOT_TIMINGS

//monitoring_Task
void monitoring_Task()
{
  tTemp = micros();
    
  if(monitor)
  {
    Serial.print ("cons_ext.:");
    Serial.print (_externalSetpoint, 1);
    Serial.print (",MainFanRpm:");
    Serial.print (_mainFanSpeed, 0);
    Serial.print (",SecondaryFanRpm:");
    Serial.print (_secondaryFanSpeed, 0);
    Serial.print (",setpointRPM:");
    Serial.print (setpointRPM, DEC);
    Serial.print (",HauteurCM:");
    Serial.print (plotHeightCm,1);
    Serial.print (",HauteurPercent:");
    Serial.print (plotHeightPercent,1);
    Serial.print (",Erreur de position:");
    Serial.print (erreur_pos,1);
    Serial.print (",SetpointPos:");
    Serial.print (_lastTrajSetpoint,1);
    Serial.print ("\r\n");
  }

#ifdef PLOT_TIMINGS
  Serial.println ("Application timings: ");
  Serial.println ("Speed Task: "+String(float(tExecSpeedTask)/1000)+" ms");
  Serial.println ("Pos Task: "+String(float(tExecPosTask)/1000)+" ms");
  Serial.println ("User Task: "+String(float(tExecUserTask)/1000)+" ms");
  Serial.println ("Monitoring Task: "+String(float(tExecMonTask)/1000)+" ms");
#endif

    tExecMonTask = micros() - tTemp;
}

#define SPEED_TASK_PERIOD_MS 43
#define POS_TASK_PERIOD_MS 11
#define MONITORING_TASK_PERIOD_MS 100

//Calcul de la période de la tâche
#define TIMER_TASK_MS (SPEED_TASK_PERIOD_MS+POS_TASK_PERIOD_MS)

// the soft timer object
SimpleTimer timerTask;
SimpleTimer timerMonitor;
SimpleTimer timerSpeed;
int counterHard=0;
int counterSoft=0;

void HandlerTickTask()
{
  //No print inside these
  position_Ctrl_Task();
}

void HandlerTickMonitor() {
  user_Ctrl_Task();
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

  timerTask.setInterval(TIMER_TASK_MS, HandlerTickTask); 
  timerMonitor.setInterval(MONITORING_TASK_PERIOD_MS, HandlerTickMonitor);
  timerSpeed.setInterval(SPEED_TASK_PERIOD_MS, speed_Ctrl_Task);
  
  Serial.begin(115200);
}

void loop() 
{ 

  //Serial.println("Temps d'execution speed: "+String(tExecSpeedTask));
  //Serial.println("Temps d'execution position: "+String(tExecPosTask)); 

  timerTask.run();
  timerMonitor.run();
  timerSpeed.run();
}
