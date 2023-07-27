#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <DeltaKinematics.h>

 

#define SERVOMIN  115 // Minimum value
#define SERVOMAX  530  // Maximum value
#define Proximity_SENSOR_PIN 18
#define RELAY_PIN 5
#define RELAY_PIN2 19





Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
DeltaKinematics DK(70,300,139,112);
TaskHandle_t Task1;
TaskHandle_t Task2;

unsigned long  previousMillis = 0;
unsigned long  previousMillis1 = 0;
const long interval = 10;
const long interval2 = 1500;
bool runn_once = true;
byte p=0; // sequence step180
byte machine_step=0;
short delta_x_pos[]={};
short delta_y_pos[]={};
short delta_z_pos[]={};


uint8_t pca_servo_ports[4]={0,4,8,12};
byte myservo_courrent[4] ={90,90,90,90};
unsigned short int pwm_signal[4]={1,1,1,1};
int val[4]={90,90,90,90};  
bool servo_arvived_todestination[4]={true,true,true,true};
byte Servo_sequence[]= {0,40,0,80,0,120,0,160,0,180};
byte Servo1_sequence[]={0,40,0,80,0,120,0,160,0,180};
byte Servo2_sequence[]={0,40,0,80,0,120,0,160,0,180};
byte Servo3_sequence[]={0,40,0,80,0,120,0,160,0,180};

/*<-----Funksionet----->*/


void conveyer_relay_on(){
  digitalWrite(RELAY_PIN,HIGH);
}
void conveyer_relay_off(){
  digitalWrite(RELAY_PIN,LOW);
}


void Delta_robot_kinematic_logic(){

  for (size_t i = 0; i <  sizeof(delta_x_pos) / sizeof(short); i++)
  {
    DK.inverse(delta_x_pos[i],delta_y_pos[i],delta_z_pos[i]);
    delay(200);
  }
  
  

}

/*<-----Funksionet----->*/

void Task2code( void * pvParameters ){
  

  for(;;){



    vTaskDelay(10);  //https://rntlab.com/question/error-task-watchdog-got-triggered/
  
  }
}



void Task1code( void * pvParameters ){
  

  for(;;){
  
    vTaskDelay(10);               //https://rntlab.com/question/error-task-watchdog-got-triggered/
  unsigned long currentMillis = millis();



switch(machine_step){
   case 0 :{ /*---------------------------------------------------------------------------------------------------*/

        bool state = digitalRead(Proximity_SENSOR_PIN);

               if (state == LOW){
               
                conveyer_relay_on();
                machine_step++;
             }else{
                  
                  conveyer_relay_off();
             }
           
 
   }break;/*---------------------------------------------------------------------------------------------------*/








    case 1 :{

      /*---------------------------------------------------------------------------------------------------*/

      if(p>=10){p=0;machine_step++;}
  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;
    for (byte i = 0; i < 4; i++)
  {
 
      if(myservo_courrent[i]-Servo_sequence[p]>0){
        myservo_courrent[i]--;
        servo_arvived_todestination[i]=false;


      }else if(myservo_courrent[i]-Servo_sequence[p]<0){

        myservo_courrent[i]++;
        servo_arvived_todestination[i]=false;
      }else{

              myservo_courrent[i]=myservo_courrent[i];
              servo_arvived_todestination[i]=true;
      }   
  }
 }

for (byte i = 0; i < 4; i++)
  {
    
    pwm_signal[i]=map( myservo_courrent[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
  }


if(servo_arvived_todestination[0]&&servo_arvived_todestination[1]&&servo_arvived_todestination[2]&&servo_arvived_todestination[3]){
  p++;
      Serial.println("------------------");
            delay(500);
      for(byte i=0;i<4;i++){
        
        Serial.println(myservo_courrent[i]);    
      }
}


/*---------------------------------------------------------------------------------------------*/

      }break;





    case 2 :{


         bool state = digitalRead(Proximity_SENSOR_PIN);

               if (state == LOW){
               
               conveyer_relay_on();
                digitalWrite(LED_BUILTIN,HIGH);
                machine_step++;
                Serial.println("on");
             }else
             {
              conveyer_relay_off();
              Serial.println("off");
              machine_step=2;
             }
             
       
    }break;

    default:{
       machine_step=0;
      
      
    }


}

    






//------------------ending bracket----------------------------
  } 
}





void setup() { 
  
  Serial.begin(115200);
  pca9685.begin();
  pinMode(Proximity_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  val[0]= myservo_courrent[0];
   val[1]= myservo_courrent[1];
   val[2]= myservo_courrent[2];
   val[3]= myservo_courrent[3];
   for (byte i = 0; i < 4; i++)
  {
    
    pwm_signal[i]=map(val[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
  }

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);
   
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}






void loop() {
  
}