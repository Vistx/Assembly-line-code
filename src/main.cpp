#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
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
PCF8574 pcf8574(0x26);
TaskHandle_t Task1;
TaskHandle_t Task2;
decode_results results;
Modbus slave(1,Serial,0);

unsigned long  previousMillis = 0;
unsigned long  previousMillis1 = 0;
unsigned long  previousMillis2 = 0;
const long interval = 35;
const long interval2 = 1500;
const long interval_2dof = 50;

bool runn_once = true;
int memory_address=0;
byte p=0,    ir_arr_pos=0; // sequence,step180
byte servo_sequence_count_eeprom=0;
int Sequence_eeprom_memory_addr=32000; 
byte get_last_step_from_eeprom=0;
byte machine_step=1;
byte value[]={90,90,90,90,0,0};// ir value array
uint16_t au16data[16] = {0, 0, 0, 0, 0, 0, 0, 90, 90, 90, 90, 0, 0, 0, 1, 1};
byte pca_servo_ports[6]={0,4,8,12,14,15};
byte _2dof_pca_servo_ports[2]={1,2};
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

byte _2dof_servo_sequence0[]={80,90,100,90};
byte _2dof_servo_sequence1[]={80,90,100,90};
bool _2dof_servo_arvived_todestination[2]={true,true};
byte myservo_courrent_2dof[2]={90,90};
byte _2dof_step=0;
/*<-----Funksionet----->*/


void conveyer_relay_on(){
   pcf8574.digitalWrite(P0, LOW); //Low becaouse of I2c Binay composition
}

void conveyer_relay_off(){
pcf8574.digitalWrite(P0, HIGH);
}


void Pump_1 (String _state ){

  if(_state=="on"){pcf8574.digitalWrite(P1, LOW);au16data[0]=1;}
  if(_state=="off"){pcf8574.digitalWrite(P1, HIGH);au16data[0]=0;}
  
}
void Pump_2 (String _state ){

  if(_state=="on"){pcf8574.digitalWrite(P2, LOW);au16data[1]=1;}
  if(_state=="off"){pcf8574.digitalWrite(P2, HIGH);au16data[1]=0;}
  
}


void writeEEPROM(int address, byte val, int i2c_address) //https://dronebotworkshop.com/eeprom-arduino/
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
       case 0x80 :{ //ir codes decoded from the remote
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Saved To memory" );
      lcd.setCursor(0,1);
      lcd.print("Adress: "+ String(memory_address)+ "-" +String(memory_address+6) );
      delay(1500);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("S"+String(p)+": ");
      digitalWrite(LED_BUILTIN,HIGH);
      if (memory_address<150){ //save to eeprom array
        for (size_t i = 0; i < 6; i++)
        {
          writeEEPROM(memory_address+i , value[i], EEPROM_I2C_ADDRESS);
        } 
        memory_address=memory_address+6;
        servo_sequence_count_eeprom=servo_sequence_count_eeprom+1;

      }else{
        
      }
      
      delay(50);
      digitalWrite(LED_BUILTIN,LOW);

       }
       break;
       case 0x2A35 :{
        if (value[ir_arr_pos]<180)
        {
          value[ir_arr_pos]=value[ir_arr_pos]+1;
          delay(20);
        }
        if (ir_arr_pos==5)
        {
          value[ir_arr_pos]=180;
        }
        if (ir_arr_pos==4)
        {
          value[ir_arr_pos]=115;
        }

       }break;

      case 0x2A36 :{
        if (value[ir_arr_pos]>0)
        {
          value[ir_arr_pos]=value[ir_arr_pos]-1;
          delay(20);
        }

         if (ir_arr_pos==5)
        {
          value[ir_arr_pos]=0;
        }

if (ir_arr_pos==4)
        {
          value[ir_arr_pos]=0;
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
      case 0x2A5A :{
        writeEEPROM(Sequence_eeprom_memory_addr, servo_sequence_count_eeprom, EEPROM_I2C_ADDRESS);
         lcd.clear();
       lcd.setCursor(0,0);
      lcd.print("Seq saved:");
      lcd.setCursor(0,1);
      lcd.print(String(servo_sequence_count_eeprom));
           
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
  au16data[14]=ir_arr_pos;


      for (byte i = 0; i < 6; i++)
  {

 
    if (i==2 || i==3 )
    {
      pwm_signal[i]=map(180-value[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
    }else{
 
    pwm_signal[i]=map(value[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);}


  } // if (IrReceiver.decodedIRData.decodedRawData==0x80){}        
        IrReceiver.resume();
  }

}

void _2dof_servo_movement(byte i , byte *_2dof_servo_sequence){

  if( myservo_courrent_2dof[i]-*_2dof_servo_sequence>0){
         myservo_courrent_2dof[i]--;
        _2dof_servo_arvived_todestination[i]=false;


      }else if( myservo_courrent_2dof[i]-*_2dof_servo_sequence<0){

         myservo_courrent_2dof[i]++;
        _2dof_servo_arvived_todestination[i]=false;
      }else{
              _2dof_servo_arvived_todestination[i]=true;
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

void Task2code( void * pvParameters ){ //commuicate with the RTU
  

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
  unsigned long currentMillis1 = millis();


  if (runn_once)
  {
   bool buttonState = digitalRead(33);
  if (buttonState){machine_step=0; au16data[13]=1; }
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

    case(2):{

      if(_2dof_step>=sizeof(&_2dof_servo_sequence0)){
        _2dof_step=0;
        au16data[5]=0;
        conveyer_relay_on();
        au16data[2]=1;
        machine_step++;}

        
  if (currentMillis1 - previousMillis2 >= interval_2dof) {previousMillis2 = currentMillis1;
  _2dof_servo_movement(0 , &_2dof_servo_sequence0[_2dof_step]);
  _2dof_servo_movement(1 , &_2dof_servo_sequence1[_2dof_step]);}


  for (byte i = 0; i < 2; i++)
  {
    pca9685.setPWM(_2dof_pca_servo_ports[i], 0, map(myservo_courrent_2dof[i], 0, 180, SERVOMIN, SERVOMAX));
    // Serial.println(myservo_courrent_2dof[i]);
  }
  au16data[11]=myservo_courrent_2dof[0]-90;
  au16data[12]=myservo_courrent_2dof[1]-90;


  if(_2dof_servo_arvived_todestination[0]&&_2dof_servo_arvived_todestination[1]){
  for(byte i=0;i<2;i++){
        
        _2dof_servo_arvived_todestination[i]=false;    
      }
  
  _2dof_step++;
   Pump_1("on");
   Pump_2("on");
   delay(900);
   Pump_1("off");
   Pump_2("off");
  
  }
    }break;

    case 3 :{

        byte state = digitalRead(26);

               if (state == 1){
                
                
                au16data[6]=0;
                conveyer_relay_on();
                au16data[2]=1;
              
                
             }else{
              au16data[2]=0;
              conveyer_relay_off();
              au16data[6]=1;
              machine_step++;
             

             }
             
       
    }break;

case 4 :{

      /*---------------------------------------------------------------------------------------------------*/

      if(p>=get_last_step_from_eeprom){
        p=0;
        au16data[5]=0;
        conveyer_relay_on();
        au16data[2]=1;

        machine_step++;
        
        
        }
      
      
      if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      _4dof_servo_movement(0,&Servo_sequence[p]);
      _4dof_servo_movement(1,&Servo1_sequence[p]);
      _4dof_servo_movement(2,&Servo2_sequence[p]);
      _4dof_servo_movement(3,&Servo3_sequence[p]);
         myservo_courrent[4]=Pump_sequence[p];
         myservo_courrent[5]=Solenoid_sequence[p];
      }

//Comunicate servo positions to the Software
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
    
    if (i==2 || i==3 )
    {
      pwm_signal[i]=map(180-myservo_courrent[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);
    }else{
 
    pwm_signal[i]=map(myservo_courrent[i], 0, 180, SERVOMIN, SERVOMAX);
    pca9685.setPWM(pca_servo_ports[i], 0, pwm_signal[i]);


    }
  }

///verify if the 4dof servos have rached the position
if(servo_arvived_todestination[0]&&servo_arvived_todestination[1]&&servo_arvived_todestination[2]&&servo_arvived_todestination[3]){
  for(byte i=0;i<4;i++){
        
        servo_arvived_todestination[i]=false;    
      }
  
  p++;
     
            delay(500);
      for(byte i=0;i<6;i++){//for debuging  purpose
      }
}
/*---------------------------------------------------------------------------------------------*/
     }break;
    default:{
       
      vTaskDelay(10);
      
    }
}
//------------------ending bracket----------------------------
  } 
}

void setup() { 
  
  Serial.begin(115200);
  slave.start();
  pcf8574.pinMode(P0, OUTPUT); //i2c demultiplex pins
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);
  pcf8574.pinMode(P3, OUTPUT);

  pcf8574.begin();
  pca9685.begin();
  IrReceiver.begin(PIN_RECV);
  lcd.init();
  pinMode(33, INPUT_PULLDOWN); //Microcontroller Mode

  lcd.backlight();
  lcd.setCursor(0,0);

  lcd.print("Runnging on ");
  lcd.setCursor(0,1);
  lcd.print("SCADA Mode ");
 // lcd.print("S"+String(ir_arr_pos)+": ");


  pinMode(Proximity_SENSOR_PIN, INPUT);
  pinMode(26, INPUT);
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

   for (byte i = 0; i < 2; i++)
  {
    pca9685.setPWM(_2dof_pca_servo_ports[i], 0, map(myservo_courrent_2dof[i], 0, 180, SERVOMIN, SERVOMAX));
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

get_last_step_from_eeprom=readEEPROM(Sequence_eeprom_memory_addr,EEPROM_I2C_ADDRESS);

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);
   //https://wokwi.com/projects/384350947215370241
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