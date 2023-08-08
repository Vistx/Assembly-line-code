#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <DeltaKinematics.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

 
#define PWM_Module_I2C 0X40
#define EEPROM_I2C_ADDRESS 0x50
#define SERVOMIN  115 // Minimum value
#define SERVOMAX  530  // Maximum value
#define lcd_Columns 16
#define lcd_Rows 2
#define Proximity_SENSOR_PIN 18 
#define PIN_RECV 4 //ir module recieve pin
#define RELAY_PIN 5 //relay pin
#define RELAY_PIN2 19 //relay pin


Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PWM_Module_I2C);
LiquidCrystal_I2C lcd(0x3F, lcd_Columns, lcd_Rows);
DeltaKinematics DK(70,300,139,112);
TaskHandle_t Task1;
TaskHandle_t Task2;
decode_results results;


unsigned long  previousMillis = 0;
unsigned long  previousMillis1 = 0;
const long interval = 30;
const long interval2 = 1500;
bool runn_once = true;
int menory_address=0;
byte p=0,    ir_arr_pos=0; // sequence step180
byte value[]={0,0,0,0,0,0};// ir value array


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
byte Servo2_sequence[]={0,20,0,80,0,135,0,160,0,180};
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

void writeEEPROM(int address, byte val, int i2c_address)
{
  // Begin transmission to I2C EEPROM
  Wire.beginTransmission(i2c_address);
 
  // Send memory address as two 8-bit bytes
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
 
  // Send data to be stored
  Wire.write(val);
 
  // End the transmission
  Wire.endTransmission();
 
  // Add 5ms delay for EEPROM
  delay(5);
}

byte readEEPROM(int address, int i2c_address)
{
  // Define byte for received data
  byte rcvData = 0xFF;
 
  // Begin transmission to I2C EEPROM
  Wire.beginTransmission(i2c_address);
 
  // Send memory address as two 8-bit bytes
  Wire.write((int)(address >> 8));   // MSB
  Wire.write((int)(address & 0xFF)); // LSB
 
  // End the transmission
  Wire.endTransmission();
 
  // Request one byte of data at current memory address
  Wire.requestFrom(i2c_address, 1);
 
  // Read the data and assign to variable
  rcvData =  Wire.read();
 
  // Return the data as function output
  return rcvData;
}
   
void Ir_remote_programming(){
     lcd.setCursor(0, 1);
     lcd.print(value[ir_arr_pos]);
     lcd.print(" ");  // to remove 0 bug


if (IrReceiver.decode()){
        // Serial.println(IrReceiver.decodedIRData.decodedRawData,HEX);
        Serial.println("Courrent Servo"+ String(p)+" value:" + String(value[ir_arr_pos]));
        
   


       switch (IrReceiver.decodedIRData.decodedRawData)
       {
       case 0x80 :{

      Serial.println("Button is pressed");
      Serial.println("Saved Servo Values: " + String(value[ir_arr_pos]));
      Serial.println("Courrent Saving addr:" + String(menory_address) + "-" +String(menory_address+6));
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Saved To memory" );
      lcd.setCursor(0,1);
      lcd.print("Adress: "+ String(menory_address)+ "-" +String(menory_address+6) );
      delay(1500);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("S"+String(p)+": ");
      digitalWrite(LED_BUILTIN,HIGH);
      if (menory_address<150){
        for (size_t i = 0; i < 6; i++)
        {
          writeEEPROM(menory_address+i , value[i], EEPROM_I2C_ADDRESS);
        } 
        menory_address=menory_address+6;
      }else{
        Serial.println("Memory filled from 0-149");
      }
      
      delay(250);
      digitalWrite(LED_BUILTIN,LOW);

       }
       break;
       case 0x2A12 :{
        if (value[ir_arr_pos]<180)
        {
          value[ir_arr_pos]=value[ir_arr_pos]+1;
          delay(150);
        }
       }break;

      case 0x2A13 :{
        if (value[ir_arr_pos]>0)
        {
          value[ir_arr_pos]=value[ir_arr_pos]-1;
          delay(150);
        }
      }break;
      case 0x2A33 :{
       if (ir_arr_pos<5)
       {
        ir_arr_pos++;
        delay(250);
       }
       lcd.clear();
       lcd.setCursor(0,0);
      lcd.print("S"+String(ir_arr_pos)+": ");
        
      }break;

      case 0x2A34 :{
        if (ir_arr_pos>0)
        {
          ir_arr_pos--;
          delay(250);
        }
         lcd.clear();
       lcd.setCursor(0,0);
      lcd.print("S"+String(ir_arr_pos)+": ");
      }break;




       default:
        break;
       }

                                                            // if (IrReceiver.decodedIRData.decodedRawData==0x80){}
        
        IrReceiver.resume();
  }




}

   
   void _4dof_servo_movement(byte i,byte* _Servo_sequence ){ //https://www.tutorialspoint.com/how-to-pass-an-array-by-reference-in-cplusplus
    if(myservo_courrent[i]-*_Servo_sequence>0){
        myservo_courrent[i]--;
        servo_arvived_todestination[i]=false;


      }else if(myservo_courrent[i]-*_Servo_sequence<0){

        myservo_courrent[i]++;
        servo_arvived_todestination[i]=false;
      }else{
              servo_arvived_todestination[i]=true;
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

      _4dof_servo_movement(0,&Servo_sequence[p]);
      _4dof_servo_movement(1,&Servo1_sequence[p]);
      _4dof_servo_movement(2,&Servo2_sequence[p]);
      _4dof_servo_movement(3,&Servo3_sequence[p]);
      }


for (byte i = 0; i < 4; i++)
  {
    
    pwm_signal[i]=map( myservo_courrent[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
  }


if(servo_arvived_todestination[0]&&servo_arvived_todestination[1]&&servo_arvived_todestination[2]&&servo_arvived_todestination[3]){
  for(byte i=0;i<4;i++){
        
        servo_arvived_todestination[i]=false;    
      }
  
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
  IrReceiver.begin(PIN_RECV);
  lcd.init();


  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("S"+String(ir_arr_pos)+": ");


  pinMode(Proximity_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  for (byte i = 0; i < 4; i++)
  {
  val[i]= myservo_courrent[i];
  }
  
  
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