#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <DeltaKinematics.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>
#include <ModbusRtu.h>
#include "PCF8574.h"

 
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
PCF8574 pcf8574(0x26);
TaskHandle_t Task1;
TaskHandle_t Task2;
decode_results results;
Modbus slave(1,Serial,0);

unsigned long  previousMillis = 0;
unsigned long  previousMillis1 = 0;
const long interval = 30;
const long interval2 = 1500;
bool runn_once = true;
int menory_address=0;
byte p=0,    ir_arr_pos=0; // sequence step180
byte value[]={90,90,90,90,0,0};// ir value array



byte machine_step=1;
short delta_x_pos[]={};
short delta_y_pos[]={};
short delta_z_pos[]={};

uint16_t au16data[16] = {0, 0, 0, 0, 0, 0, 0, 90, 90, 90, 90, 0, 0, 0, 1, 1};
byte pca_servo_ports[6]={0,4,8,12,14,15};
byte myservo_courrent[6] ={90,90,90,90,0,0};
unsigned short int pwm_signal[6]={1,1,1,1,1,1};
int val[6]={90,90,90,90,0,0};  
bool servo_arvived_todestination[4]={true,true,true,true};
byte Servo_sequence[25]=   {};
byte Servo1_sequence[25]=  {};
byte Servo2_sequence[25]=  {};
byte Servo3_sequence[25]=  {};
byte Pump_sequence[25]=    {};
byte Solenoid_sequence[25]={};

/*<-----Funksionet----->*/


void conveyer_relay_on(){
   pcf8574.digitalWrite(P0, LOW);
   

}
void conveyer_relay_off(){
  
pcf8574.digitalWrite(P0, HIGH);
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
     lcd.setCursor(0, 0);
     lcd.print("Servo["+String(ir_arr_pos)+ "] Programming");
     lcd.setCursor(0, 1);
     lcd.print(value[ir_arr_pos]);
     lcd.print("         Mode  ");  // to remove 0 bug


if (IrReceiver.decode()){
        // Serial.println(IrReceiver.decodedIRData.decodedRawData,HEX)
       switch (IrReceiver.decodedIRData.decodedRawData)
       {
       case 0x80 :{
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
        
      }
      
      delay(50);
      digitalWrite(LED_BUILTIN,LOW);

       }
       break;
       case 0x2A12 :{
        if (value[ir_arr_pos]<180)
        {
          value[ir_arr_pos]=value[ir_arr_pos]+1;
          delay(50);
        }
       }break;

      case 0x2A13 :{
        if (value[ir_arr_pos]>0)
        {
          value[ir_arr_pos]=value[ir_arr_pos]-1;
          delay(50);
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
for (byte i = 0; i < 4; i++)
  {
    au16data[i+7]=value[i];
  }

  au16data[3]=value[4];
  au16data[4]=value[5];



      for (byte i = 0; i < 6; i++)
  {

    
    pwm_signal[i]=map( value[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
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
    
  slave.poll( au16data, 16 );
  
    vTaskDelay(10);  //https://rntlab.com/question/error-task-watchdog-got-triggered/
  
  }
}



void Task1code( void * pvParameters ){
  

  for(;;){
  //Ir_remote_programming();
    vTaskDelay(10);               //https://rntlab.com/question/error-task-watchdog-got-triggered/
  unsigned long currentMillis = millis();

  if (runn_once)
  {
   bool buttonState = digitalRead(33);
  if (buttonState){machine_step=0;}
  runn_once=false;
  }
   


switch(machine_step){
case 0:{
 Ir_remote_programming();
}break;


   case 1 :{ /*---------------------------------------------------------------------------------------------------*/
        byte state = digitalRead(Proximity_SENSOR_PIN);
       
               if (state == 1){
                       
               au16data[5]=0;
                conveyer_relay_on();
                au16data[2]=1;
                
             }else{
                 

                  au16data[2]=0;
                  conveyer_relay_off();
                  au16data[5]=1;
                  machine_step++;
             }
           
 
   }break;/*---------------------------------------------------------------------------------------------------*/








    case 2 :{

      /*---------------------------------------------------------------------------------------------------*/

      if(p>=10){p=0;machine_step++;}
      if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      _4dof_servo_movement(0,&Servo_sequence[p]);
      _4dof_servo_movement(1,&Servo1_sequence[p]);
      _4dof_servo_movement(2,&Servo2_sequence[p]);
      _4dof_servo_movement(3,&Servo3_sequence[p]);
         myservo_courrent[4]=Pump_sequence[p];
         myservo_courrent[5]=Solenoid_sequence[p];
      }


for (byte i = 0; i < 6; i++)
  {
    if (i<4)
    {
      au16data[i+7]=myservo_courrent[i];
    }
    if (i==4)
    {
       au16data[3]=myservo_courrent[4];
    }
    if (i==5)
    {
      au16data[4]=myservo_courrent[5];
    }
    
    
    
    pwm_signal[i]=map( myservo_courrent[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
  }


if(servo_arvived_todestination[0]&&servo_arvived_todestination[1]&&servo_arvived_todestination[2]&&servo_arvived_todestination[3]){
  for(byte i=0;i<4;i++){
        
        servo_arvived_todestination[i]=false;    
      }
  
  p++;
     
            delay(500);
      for(byte i=0;i<6;i++){
        
           
      }
}


/*---------------------------------------------------------------------------------------------*/
     }break;





    case 3 :{


        byte state = digitalRead(Proximity_SENSOR_PIN);

               if (state == 1){
                
                
                au16data[5]=0;
                conveyer_relay_on();
                au16data[2]=1;
              
                
             }else{
              au16data[2]=0;
              conveyer_relay_off();
              au16data[5]=1;
              machine_step++;
             

             }
             
       
    }break;

    case(4):{

      



    }break;

    case(100):{


    }break;


    default:{
       
      
      
    }


}

    






//------------------ending bracket----------------------------
  } 
}





void setup() { 
  
  Serial.begin(115200);
  slave.start();
  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);
  pcf8574.pinMode(P3, OUTPUT);

  pcf8574.pinMode(P5,INPUT);
  pcf8574.pinMode(P6,INPUT);

  pcf8574.begin();
  pca9685.begin();
  IrReceiver.begin(PIN_RECV);
  lcd.init();
  pinMode(33, INPUT_PULLDOWN);

  lcd.backlight();
  lcd.setCursor(0,0);

  lcd.print("Runnging on ");
  lcd.setCursor(0,1);
  lcd.print("SCADA Mode ");
 // lcd.print("S"+String(ir_arr_pos)+": ");


  pinMode(Proximity_SENSOR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  for (byte i = 0; i < 4; i++)
  {
  val[i]= myservo_courrent[i];
  }
 
   for (byte i = 0; i < 6; i++)
  {
    
    pwm_signal[i]=map(val[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
  
  
  }


  
  
byte j=0;
for (size_t i = 0; i < 25; i++)
{
  
   Servo_sequence[i]= readEEPROM(j,EEPROM_I2C_ADDRESS);
    Servo1_sequence[i]= readEEPROM(j+1,EEPROM_I2C_ADDRESS);
    Servo2_sequence[i]= readEEPROM(j+2,EEPROM_I2C_ADDRESS);
    Servo3_sequence[i]= readEEPROM(j+3,EEPROM_I2C_ADDRESS);
    Pump_sequence[i]= readEEPROM(j+4,EEPROM_I2C_ADDRESS);
    Solenoid_sequence[i]=readEEPROM(j+5,EEPROM_I2C_ADDRESS);
  
  j=j+6;
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