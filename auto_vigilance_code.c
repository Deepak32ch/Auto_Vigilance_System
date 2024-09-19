 //sender phone number with country code
const String PHONE = "+91xxxxxxxxxx";

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <Adafruit_Sensor.h>    // Adafruit  sensor library
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();   // ADXL345 Object
Adafruit_SH1106 display(21, 22);

#define OLED_SDA 21
#define OLED_SCL 22
#define BUTTON_PIN 25

#define DOOR_SENSOR_PIN  12  // ESP32 pin GIOP23 connected to the OUTPUT pin of door sensor
#define LED_PIN          26  // ESP32 pin GIOP17 connected to LED's pin

/*****************************************************************/
//GSM Module RX pin to ESP32 Pin 2
//GSM Module TX pin to ESP32 Pin 4
#define rxPin 4
#define txPin 2
#define BAUD_RATE 9600
HardwareSerial sim800(1);
/****************************************************************/

int doorState;
int motor1Pin1 = 18; 
int motor1Pin2 = 05; 
int enable1Pin = 19;
int i =0,value =1, not_start =0;
int authorised_value =0;
/*******************for push button**********************************/
// Variables will change:
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin

/********************************************************************/

// Setting PWM properties
const int freq = 20000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;

String smsStatus,senderNumber,receivedDate,msg;
boolean isReply = false;

void setup()   {  
    Serial.begin(9600); 

    pinMode(BUTTON_PIN, INPUT_PULLUP);

  /* initialize OLED with I2C address 0x3C */
  display.begin(SH1106_SWITCHCAPVCC, 0x3C); 
  display.clearDisplay();
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(LED_PIN, OUTPUT);               // set ESP32 pin to output mode

    // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);
  
  //FOR accelerometer
    if(!accel.begin())   // if ASXL345 sensor not found
  {
    Serial.println("ADXL345 not detected");
    while(1);
  }

    /***************************************************************/
 
  Serial.println("esp32 serial initialize");
  
  sim800.begin(BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  Serial.println("SIM800L serial initialize");
  
  smsStatus = "";
  senderNumber="";
  receivedDate="";
  msg="";

  sim800.print("AT+CMGF=1\r"); //SMS text mode
  delay(1000);
  /***************************************************************/ 

}
void loop() {

  /*********************************************************/
  while(sim800.available()){
  parseData(sim800.readString());
}

while(Serial.available())  {
  sim800.println(Serial.readString());
}

/*****************************************************************/ 

  /* set text size, color, cursor position, 
  set buffer with  Hello world and show off*/
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("    C-DAC\n\n    AUTOWATCH\n    VIGILANCE\n");
  display.display();
  delay(3000);
  display.clearDisplay();



  doorState = digitalRead(DOOR_SENSOR_PIN); // read 
  /*****************************for push button*****************/
    // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

 if(currentState == LOW && doorState == HIGH)
  {
    
    authorised();
   
  }


 else if (lastState ==  LOW && currentState == HIGH && doorState == HIGH && authorised_value == 0 && not_start == 0)
  
   { 
     unauthorised();
   }
  else if (doorState == LOW && (i==0 || i==1) )
  {
     closeDoor();
  }
    // save the the last state
  //lastState = currentState;
  else if(authorised_value == 1)
  {
    sensors_event_t event;
    accel.getEvent(&event);
    display.print("X: ");
    display.print(event.acceleration.x);
    display.print("  ");
    display.print("Y: ");
    display.print(event.acceleration.y);
    display.print("  ");
    display.print("Z: ");
    display.print(event.acceleration.z);
    display.print("  ");
    display.println("m/s^2 ");
    delay(500);
  }
}

void authorised()
{
    authorised_value = 1 ;
    //Serial.println("The button is pressed");
    i =1;
    display.println("--> Authorised Access\n");
    display.display();
    delay(2000);
    display.clearDisplay();
    digitalWrite(LED_PIN, LOW); // turn off LED

    // for motor
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, dutyCycle);
    // sensors_event_t event;
    // accel.getEvent(&event);
    // if(event.acceleration.x > 4 || event.acceleration.y > 4|| event.acceleration.z > 14 ||
    //   event.acceleration.x < -4 || event.acceleration.y < -4 || event.acceleration.z < 12)
    //   {
    //     Serial.println("unsafed driving");
    //    // Reply("unsafed driving");
    //   }

}

void unauthorised()
{
    i = 2;
    if(value ==1)
    {
    Reply("Unauthorised Access");
    display.println("--> sending SMS\n");
    value++;
    }
    //Serial.println("The button is released");
    
    display.println("--> Unauthorised Access\n");
    display.println("--> someone trying to access the vehicle\n");
    display.display();
    delay(2000);
    display.clearDisplay();
        // for motor
    digitalWrite(LED_PIN, HIGH); // turn on LED
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, dutyCycle);

}

void closeDoor()
{
      //Serial.println("The door is closed, turns off LED");
    digitalWrite(LED_PIN, LOW);  // turn off LED
    display.println("-->The Door is closed\n");
    display.println("-->Vehicle is in safe Zone\n");
    display.display();
    delay(2000);
    display.clearDisplay();
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    authorised_value = 0;
}

void parseData(String buff){
  Serial.println(buff);

  unsigned int len, index;


  index = buff.indexOf("\r");
  buff.remove(0, index+2);
  buff.trim();

  if(buff != "OK"){
    index = buff.indexOf(":");
    String cmd = buff.substring(0, index);
    cmd.trim();
    
    buff.remove(0, index+2);
    
    if(cmd == "+CMTI"){
 
      index = buff.indexOf(",");
      String temp = buff.substring(index+1, buff.length()); 
      temp = "AT+CMGR=" + temp + "\r"; 

      sim800.println(temp); 
    }
    else if(cmd == "+CMGR"){
      extractSms(buff);
      
      
      if(senderNumber == PHONE){
        doAction();
      }
    }

  }
  else{

  }
}


void extractSms(String buff){
   unsigned int index;
   
    index = buff.indexOf(",");
    smsStatus = buff.substring(1, index-1); 
    buff.remove(0, index+2);
    
    senderNumber = buff.substring(0, 13);
    buff.remove(0,19);
   
    receivedDate = buff.substring(0, 20);
    buff.remove(0,buff.indexOf("\r"));
    buff.trim();
    
    index =buff.indexOf("\n\r");
    buff = buff.substring(0, index);
    buff.trim();
    msg = buff;
    buff = "";
    msg.toLowerCase();
}

void doAction(){

  if(msg == "engine off"){
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, dutyCycle);
    Reply("vehicle safe");
    value--;
    not_start = 1;
  }
  else if(msg == "relay2 on"){
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, dutyCycle);
    Reply("Relay 2 has been ON");
  }

  
  smsStatus = "";
  senderNumber="";
  receivedDate="";
  msg="";  
}

void Reply(String text)
{
    sim800.print("AT+CMGF=1\r");
    delay(1000);
    sim800.print("AT+CMGS=\""+PHONE+"\"\r");
    delay(1000);
    sim800.print(text);
    delay(100);
    sim800.write(0x1A); 
    delay(1000);
    Serial.println("SMS Sent Successfully.");
}






