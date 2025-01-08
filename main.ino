// cd C:\Program Files\mosquitto
// mosquitto -v -c testconfig.txt
// mosquitto_pub -t esp32/sub -h localhost -m "hello"
// mosquitto_sub -t esp32/sub -h localhost^X

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32

volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber; 

#define BUILTIN_LED 2
#define wifi_ssid "Lina -4G"
#define wifi_password "aleen123dhar"
#define mqtt_server "192.168.1.45"

WiFiClient espClient;
PubSubClient client(espClient);
const char* esp32_sub = "esp32/pub";


Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 18;
const int mot2_pin = 26;
const int mot3_pin = 27;
const int mot4_pin = 19;


volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6]; // Increase the array size to 6 for Channel 1 to Channel 6
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

// float voltage;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[] = {0, 0, 0};

// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch;
// float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
// float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
// float Kalman1DOutput[]={0,0};

float PRateRoll = 0.75; //For outdoor flights, keep this gain to 0.75 and for indoor flights keep the gain to be 0.6
float IRateRoll = 0.012;
float DRateRoll = 0.0085;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4.2;
float IRateYaw = 2.8;
float DRateYaw = 0;

uint32_t LoopTimer;
float t=0.006;      //time cycle

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;
float PAngleRoll=2; float PAnglePitch=PAngleRoll;
float IAngleRoll=0; float IAnglePitch=IAngleRoll;
float DAngleRoll=0; float DAnglePitch=DAngleRoll;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s

  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// void channelInterruptHandler()
// {
//   current_time = micros();
//   // Channel 1
//   if (digitalRead(channel_1_pin))
//   {
//     if (last_channel_1 == 0)
//     {
//       last_channel_1 = 1;
//       timer_1 = current_time;
//     }
//   }
//   else if (last_channel_1 == 1)
//   {
//     last_channel_1 = 0;
//     ReceiverValue[0] = current_time - timer_1;
//   }

//   // Channel 2
//   if (digitalRead(channel_2_pin))
//   {
//     if (last_channel_2 == 0)
//     {
//       last_channel_2 = 1;
//       timer_2 = current_time;
//     }
//   }
//   else if (last_channel_2 == 1)
//   {
//     last_channel_2 = 0;
//     ReceiverValue[1] = current_time - timer_2;
//   }

//   // Channel 3
//   if (digitalRead(channel_3_pin))
//   {
//     if (last_channel_3 == 0)
//     {
//       last_channel_3 = 1;
//       timer_3 = current_time;
//     }
//   }
//   else if (last_channel_3 == 1)
//   {
//     last_channel_3 = 0;
//     ReceiverValue[2] = current_time - timer_3;
//   }

//   // Channel 4
//   if (digitalRead(channel_4_pin))
//   {
//     if (last_channel_4 == 0)
//     {
//       last_channel_4 = 1;
//       timer_4 = current_time;
//     }
//   }
//   else if (last_channel_4 == 1)
//   {
//     last_channel_4 = 0;
//     ReceiverValue[3] = current_time - timer_4;
//   }

//   // Channel 5
//   if (digitalRead(channel_5_pin))
//   {
//     if (last_channel_5 == 0)
//     {
//       last_channel_5 = 1;
//       timer_5 = current_time;
//     }
//   }
//   else if (last_channel_5 == 1)
//   {
//     last_channel_5 = 0;
//     ReceiverValue[4] = current_time - timer_5;
//   }

//   // Channel 6
//   if (digitalRead(channel_6_pin))
//   {
//     if (last_channel_6 == 0)
//     {
//       last_channel_6 = 1;
//       timer_6 = current_time;
//     }
//   }
//   else if (last_channel_6 == 1)
//   {
//     last_channel_6 = 0;
//     ReceiverValue[5] = current_time - timer_6;
//   }
// }


// void neutralPositionAdjustment()
// {
//   int min = 1490;
//   int max = 1510;
//   if (ReceiverValue[0] < max && ReceiverValue[0] > min)
//   {
//     ReceiverValue[0]= 1500;
//   } 
//   if (ReceiverValue[1] < max && ReceiverValue[1] > min)
//   {
//     ReceiverValue[1]= 1500;
//   } 
//   if (ReceiverValue[3] < max && ReceiverValue[3] > min)
//   {
//     ReceiverValue[3]= 1500;
//   } 
//   if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
//   {
//     ReceiverValue[0]= 1500;
//     ReceiverValue[1]= 1500;
//     ReceiverValue[3]= 1500;
//   }
// }


void gyro_signals(void)
{
 Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AccZ=AccZ-0.26; // calibration offset
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);

}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid(void)
{
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  char str[length+1];
  if (strcmp(topic, esp32_sub) == 0 ){
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    JsonDocument doc;
    // deserializeJson(doc, payload, length);
    // deserializeJson(doc, (char*) payload, length);
    // Serial.print(doc["speed"]);
    int i=0;
    for (i = 0; i < length; i++) {
    // Serial.print((char)payload[i]);
    str[i]=(char)payload[i];
  }
  str[i] = 0; // Null termination
  Serial.print(str);
  Serial.println();
  deserializeJson(doc,str);
  long speed1=doc["speed"][0];
  long speed2=doc["speed"][1];
  long speed3=doc["speed"][2];
  long speed4=doc["speed"][3];
  Serial.print(speed1);
  Serial.print(",");
  Serial.print(speed2);
  Serial.print(",");
  Serial.print(speed3);
  Serial.print(",");
  Serial.print(speed4);
  Serial.println();


// // 0 to 180 is clockwise
// int arr1[2] = [0,180];
// // 180 to 0 is counter clockwise
// int arr1[2] = [0,180];
  // DesiredAngleRoll=0;
  // DesiredAnglePitch=0;
  InputThrottle=speed1;
  // DesiredRateYaw=0;
  // int mmotorSpeed1 = map(speed1, 1000, 2000, 0, 180);
  // int mmotorSpeed2 = map(speed2, 1000, 2000, 0, 180);
  // int mmotorSpeed3 = map(speed3, 1000, 2000, 0, 180);
  // int mmotorSpeed4 = map(speed4, 1000, 2000, 0, 180);
  // mot1.write(mmotorSpeed1);
  // mot2.write(mmotorSpeed2);
  // mot3.write(mmotorSpeed3);
  // mot4.write(mmotorSpeed4);
  }
  // Serial.print("Message arrived [");
  // Serial.print(topic);
  // Serial.print("] ");


  // Switch on the LED if an 1 was received as first character
  // if ((char)payload[0] == '1') {
  //   digitalWrite(2, LOW);   // Turn the LED on (Note that LOW is the voltage level
  //   // but actually the LED is on; this is because
  //   // it is active low on the ESP-01)
  // } else {
  //   digitalWrite(2, HIGH); // Turn the LED off by making the voltage HIGH
  //   delay(2000);
  //   digitalWrite(2, LOW);
  // }

}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32lient-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(esp32_sub);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}



void setup(void) {
  
Serial.begin(115200);

mot1.attach(mot1_pin,1000,2000);
mot2.attach(mot2_pin,1000,2000);
mot3.attach(mot3_pin,1000,2000);
mot4.attach(mot4_pin,1000,2000);
setup_wifi();

// int led_time=100;
//  pinMode(15, OUTPUT);
//   digitalWrite(15, LOW);
//   delay(led_time);
//   digitalWrite(15, HIGH);
//   delay(led_time);
//   digitalWrite(15, LOW);
//   delay(led_time);
//   digitalWrite(15, HIGH);
//   delay(led_time);
//   digitalWrite(15, LOW);
//   delay(led_time);
//   digitalWrite(15, HIGH);
//   delay(led_time);
//   digitalWrite(15, LOW);
//   delay(led_time);
//   digitalWrite(15, HIGH);
//   delay(led_time);
//   digitalWrite(15, LOW);
//   delay(led_time);


//   pinMode(channel_1_pin, INPUT_PULLUP);
//   pinMode(channel_2_pin, INPUT_PULLUP);
//   pinMode(channel_3_pin, INPUT_PULLUP);
//   pinMode(channel_4_pin, INPUT_PULLUP);
//   pinMode(channel_5_pin, INPUT_PULLUP);
//   pinMode(channel_6_pin, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

 
//to stop esc from beeping
  mot1.write(0);
  mot2.write(0);
  mot3.write(0);
  mot4.write(0); 
  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(2000);
  digitalWrite(15, LOW);
  delay(2000);

  InputThrottle=1300;
  DesiredAngleRoll=0;
  DesiredAnglePitch=0;
  DesiredRateYaw=0;

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 4000; RateCalibrationNumber++)
  {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 4000;
  RateCalibrationPitch /= 4000;
  RateCalibrationYaw /= 4000;
// Gyro Calibrated Values
  Serial.print("Gyro Calib: ");
  Serial.print(RateCalibrationRoll);
  Serial.print("  ");
  Serial.print(RateCalibrationPitch);
  Serial.print("  ");
  Serial.print(RateCalibrationYaw);
  Serial.print(" -- ");

  //setup mqtt
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);
  digitalWrite(15, HIGH);
  delay(1000);
  digitalWrite(15, LOW);
  delay(1000);




  

LoopTimer = micros();


}

void loop(void) {

    if (!client.connected()) {
      reconnect();
    }
  client.loop();
  //enter your loop code here
  gyro_signals();
   RateRoll -= RateCalibrationRoll;
   RatePitch -= RateCalibrationPitch;
   RateYaw -= RateCalibrationYaw;
  //  Serial.print(RateRoll);
  //  Serial.print(RatePitch);
  //  Serial.print(RateYaw);
  //  Serial.println();



  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  // channelInterruptHandler();
  // neutralPositionAdjustment();

  // DesiredAngleRoll=0.1*(ReceiverValue[0]-1500);
  // DesiredAnglePitch=0.1*(ReceiverValue[1]-1500);
  // InputThrottle=ReceiverValue[2];
  // DesiredRateYaw=0.15*(ReceiverValue[3]-1500);





  ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
  ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;

  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);     
  DesiredRateRoll=PIDReturn[0]; 
  PrevErrorAngleRoll=PIDReturn[1];
  PrevItermAngleRoll=PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch=PIDReturn[0]; 
  PrevErrorAnglePitch=PIDReturn[1];
  PrevItermAnglePitch=PIDReturn[2];

  ErrorRateRoll=DesiredRateRoll-RateRoll;
  ErrorRatePitch=DesiredRatePitch-RatePitch;
  ErrorRateYaw=DesiredRateYaw-RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRateRoll=PIDReturn[1]; 
       PrevItermRateRoll=PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch,IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorRatePitch=PIDReturn[1]; 
       PrevItermRatePitch=PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];

  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise


// yoo

  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


  int ThrottleIdle = 1350;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

  // int ThrottleCutOff = 1000;
  // if (ReceiverValue[2] < 1050)
  // {
  //   MotorInput1 = ThrottleCutOff;
  //   MotorInput2 = ThrottleCutOff;
  //   MotorInput3 = ThrottleCutOff;
  //   MotorInput4 = ThrottleCutOff;
  //   reset_pid();
  // }

  mot1.write(map(MotorInput1, 1000, 2000, 0, 180));
  mot2.write(map(MotorInput2, 1000, 2000, 0, 180));
  mot3.write(map(MotorInput3, 1000, 2000, 0, 180));
  mot4.write(map(MotorInput4, 1000, 2000, 0, 180));




// voltage= (analogRead(36)/4096)*12.46*(35.9/36);
// if(voltage<11.1)
// {

// }

//Reciever signals
  // Serial.print(ReceiverValue[0]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[1]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[2]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[3]);
  // Serial.print(" --- ");
 
//   // Serial.print(ReceiverValue[4]);
//   // Serial.print(" - ");
//   // Serial.print(ReceiverValue[5]);
//   // Serial.print(" - ");

// Motor PWMs in us
  Serial.print("MotVals-");
  Serial.print(MotorInput1);
  Serial.print("  ");
  Serial.print(MotorInput2);
  Serial.print("  ");
  Serial.print(MotorInput3);
  Serial.print("  ");
  Serial.print(MotorInput4);
  Serial.print(" -- ");

// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

// //Gyro Rates
  // Serial.print(" Gyro rates:");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print(" -- ");



// // PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

// // Angles from MPU
//   Serial.print("AngleRoll:");
//   Serial.print(AngleRoll);
//   //serial.print("  ");
//     Serial.print("AnglePitch:");
//   Serial.println(AnglePitch);

  
  
//   Serial.print("KalmanAngle ");
// Serial.print(KalmanAngleRoll);
//   Serial.print("  ");
// Serial.print(KalmanUncertaintyAngleRoll);
//   Serial.print("  ");
// Serial.print(KalmanAnglePitch);
//   Serial.print("  ");
// Serial.print(KalmanUncertaintyAnglePitch);
//   Serial.print(" -- ");

//    Serial.println(" ");

  Serial.print("InputThrottle: ");
  Serial.print(InputThrottle);
  Serial.println(" ");
  while (micros() - LoopTimer < (t*1000000));
  {
     LoopTimer = micros();

  }

}

//final code