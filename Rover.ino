#include <L298NX2.h>
#include <Arduino.h>
#include <Ticker.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include "FreeRTOS.h"
#include "task.h"
#include <string>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>




//----------global ------------------------
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // Sets speed of DC motors
int speedSet = 0;
int distance = 60;
int distanceR = 0;
int distanceL = 0;
int speeed = 255;



//===============COMPONENT DEFINITIONS================//
//--------- Flag structure --------------------------------------
typedef struct _vFlag
{
  uint8_t HCSR04Flag = 1;

} vFlag;
vFlag *flag_Ptr;
vFlag flag;



//MOTORS
// Pin definition
const unsigned int IN1 = 11;
const unsigned int IN2 = 12;
const unsigned int EN = 13;
const unsigned int IN1_B = 15;
const unsigned int IN2_B = 10;
const unsigned int EN_B = 14;
// Create one motor instance
L298NX2 motors(EN, IN1, IN2, EN_B, IN1_B, IN2_B);

// Initial speed
unsigned short theSpeed = 0;

//LED
#define LED_PIN 2

//PHOTON RESISTOR
#define PHOTON_RES 4

//GYROSCOPE
#define RPM_PIN 17
Adafruit_MPU6050 mpu;

// NETWORK CREDENTIALS
const char* ssid = "Zyxel_A535E1";
const char* password = "XN4TY3EQ";
#define WIFI_CHANNEL 6

IPAddress local_IP(192, 168, 1, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(192, 168, 1, 1);
IPAddress secondaryDNS(0, 0, 0, 0);


//ECHO
#define TRIGPIN_PIN 5
#define ECHO_PIN 18
long duration;
unsigned long currentMillis = 0;

// DHT11
#define DHTPIN 0 
#define DHTTYPE DHT11
#define CONNECTION_TIMEOUT 10
DHT dht(DHTPIN, DHTTYPE);

// TASKS
TaskHandle_t hfunction;
void vFunctionTask(void *pvParameters);
void initial()
{
  Serial.println(F("Create Task"));
  //---------------------------------------------------------------------
  xTaskCreatePinnedToCore(
    vFunctionTask, "FunctionTask"
    , 
    1024 // Stack size
    ,
    NULL, 1 // Priority
    ,
    &hfunction
    , 
    1);
}
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


//===============COMPONENT FUNCTIONS================//

//DHT11 FUNCTIONS
String readDHTTemperature() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {    
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.println(t);
    return String(t);
  }
}

String readDHTHumidity() {
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.println(h);
    return String(h);
  }
}

String readDHTHeatIndex(){
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    float hic = dht.computeHeatIndex(t, h, false);
    if (isnan(hic)) {
    Serial.println("Failed to read from DHT sensor!");
    return "--";
  }
  else {
    Serial.println(hic);
    return String(hic);
  }
}

//GYROSCOPE FUNCTIONS

String readMPUAccelerationX(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
   float r = a.acceleration.x;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPUAccelerationY(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float r = a.acceleration.y;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPUAccelerationZ(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float r = a.acceleration.z;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPURotationX(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float r = g.gyro.x;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPURotationY(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float r = g.gyro.y;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPURotationZ(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
   float r = g.gyro.z;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}

String readMPUTemperature(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float r = temp.temperature;
    if (isnan(r)) {
    Serial.println("Failed to read from MPU6050 sensor!");
    return "--";
  }
  else {
    Serial.println(r);
    return String(r);
  }
}



//====================MOVEMENT FUNCTIONS===================================
void Forward(){
  // motors.setSpeedA(speeed);
  // motors.setSpeedB(speeed);
  // motors.forwardB();
  // motors.forwardA();

  motors.setSpeedA(speeed);
  motors.setSpeedB(speeed);
  motors.backwardB();
  motors.backwardA();
}
void Backward(){
  // motors.setSpeedA(speeed);
  // motors.setSpeedB(speeed);
  // motors.backwardB();
  // motors.backwardA();

  motors.setSpeedA(speeed);
  motors.setSpeedB(speeed);
  motors.forwardB();
  motors.forwardA();
}
void Left(){
  motors.setSpeedA(speeed);
  motors.setSpeedB(speeed);
  motors.backwardB();
  motors.forwardA();
}

void Right() {
  motors.setSpeedA(speeed);
  motors.setSpeedB(speeed);
  motors.backwardA();
  motors.forwardB();
}

//END MOVEMENT FUNCTIONS

//===============WEBSITE CODE================//
const char index_html[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Controls</title>
    <link rel="stylesheet" href="https://drive.google.com/uc?export=view&id=10oF3743dfpbh3H7iARP0dDMXmsDyT8DW">

</head>
<body>
    <h1>ESP32 Car Controls</h1>
    <div class="container">

            <div class="module">
                <h3>GyroScope</h3>
                <ul>
                    <li>Rotation X: <span class="value" id="GRX">%GRX%</span> rads/s</li>
                    <li>Rotation Y: <span class="value" id="GRY">%GRY%</span> rads/s</li>
                    <li>Rotation Z: <span class="value" id="GRZ">%GRZ%</span> rads/s</li>
                    <li>Temperature: <span class="value" id="GT">%GT%</span> &#8451;</li>
                </ul>
            </div>

            <div class="module">
                <h3>Thermometer</h3>
                <ul>
                    <li>Humidity: <span class="value" id="TH">%TH%</span> </li>
                    <li>Temperature: <span class="value" id="TT">%TT%</span> &#8451;</li>
                    <li>Heat index: <span class="value" id="THI">%THI%</span> &#8451;</li>
                </ul>
            </div>

            <div class="module">
                <h3>Accelerometer</h3>
                <ul>
                    <li>Accelaration X: <span class="value" id="AX">%AX%</span> m/s<sup>2</sup></li>
                    <li>Accelaration Y: <span class="value" id="AY">%AY%</span> m/s<sup>2</sup></li>
                    <li>Accelaration Z: <span class="value" id="AZ">%AZ%</span> m/s<sup>2</sup></li>
                </ul>
            </div>
            
        
    </div>
    <div class="buttons">
      <button class="btn" onclick="StartM();">Start</button>
      <button class="btn" onclick="StopM();">Stop</button>

    </div>

   <script src="https://drive.google.com/uc?export=view&id=1JttfNPl_SvW4nnkMrBOrOqURKKSu4lkD"></script>
</body>
</html>
)rawliteral";



//===============PROCESSOR FUNCTION================//
// Replaces placeholder with DHT values
String processor(const String& var){
  // Serial.println(var);
  if(var == "TT"){
    return readDHTTemperature();
  }
  else if(var == "TH"){
    return readDHTHumidity();
  }
  else if(var == "THI"){
    return readDHTHeatIndex();
  }
  else if(var == "GRX"){
    return readMPURotationX();
  }
  else if(var == "GRY"){
    return readMPURotationY();
  }
  else if(var == "GRZ"){
    return readMPURotationZ();
  }
  else if(var == "GT"){
    return readMPUTemperature();
  }
  else if(var == "AX"){
    return readMPUAccelerationX();
  }
  else if(var == "AY"){
    return readMPUAccelerationY();
  }
  else if(var == "AZ"){
    return readMPUAccelerationZ();
  }
  else if(var == "startM"){
    return "start";
  }
  else if(var == "stopM"){
    return "stop";
  }
  return String();
  
}


// ===============EXTRA LOOPS============================
void light_loop(void* pvParameters) {
 while(1){
  int LightresValue = analogRead(PHOTON_RES);
  if(LightresValue < 800 ) {
    digitalWrite (LED_PIN, HIGH);
  }
  else {
    digitalWrite (LED_PIN, LOW);
  }
 } 
}


void rpm_loop(void* pvParameters){
while(1){
  int value = digitalRead(RPM_PIN);
  Serial.println(value);
  delay(5000);
}
}

//===============SETUP================//
void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);

  // Set initial rotation direction
  // MOVEMENT MOTOR SETUP END

  pinMode(LED_PIN, OUTPUT);
  int LightresValue = analogRead(PHOTON_RES);
  pinMode(RPM_PIN, INPUT);
  pinMode(TRIGPIN_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  int timeout_counter = 0;
  

    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }




 // Connect to Wi-Fi
  WiFi.begin(ssid, password,WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(ssid);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    delay(200);
        timeout_counter++;

        // if(timeout_counter >= CONNECTION_TIMEOUT*5){
        // ESP.restart();
        
        // }
        if(timeout_counter >= CONNECTION_TIMEOUT*5){
          // pwmChannel = 3000;
          break;
        }
  }

  // WiFi.begin(ssid, password,WIFI_CHANNEL);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi..");
  // }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  // DHT11
  dht.begin();



// TASKS

initial();

 xTaskCreatePinnedToCore(
    light_loop, // function to implement task
    "light_loop", // name of task
    1000, //bytes
    NULL, // task input parameter
    0, //priority of the task
    NULL, //task handle
    0 // core of task

  );

   xTaskCreatePinnedToCore(
    rpm_loop, // function to implement task
    "rpm_loop", // name of task
    1500, //bytes
    NULL, // task input parameter
    0, //priority of the task
    NULL, //task handle
    0 // core of task
  );




// GYRO LOOP SETUP -- GOD HAVE MERCY ON MY SOUL
while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);


//GYRO LOOP SETUP END


  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/TT", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTTemperature().c_str());
  });
  server.on("/TH", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTHumidity().c_str());
  });
  server.on("/THI", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHTHeatIndex().c_str());
  });
server.on("/GRX", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPURotationX().c_str());
  });
  server.on("/GRY", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPURotationY().c_str());
  });
  server.on("/GRZ", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPURotationZ().c_str());
  });
  server.on("/GT", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPUTemperature().c_str());
  });
  server.on("/AX", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPUAccelerationX().c_str());
  });
  server.on("/AY", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPUAccelerationY().c_str());
  });
  server.on("/AZ", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readMPUAccelerationZ().c_str());
  });
  server.on("/startM", HTTP_GET, [](AsyncWebServerRequest *request){
    // start function
    Serial.println("Start");
  });
  server.on("/stopM", HTTP_GET, [](AsyncWebServerRequest *request){
    // stop function
    Serial.println("Stop");
  });
  // Start server
  server.begin();
}
 

//  =============================MAIN LOOP=================================================
void loop(){

  while(1)
  {
    if(flag.HCSR04Flag==1)
    {
      if(distance<=35)
      {
        motors.stop();
        delay(200);
        Backward();
        delay(400);
        motors.stop();
        delay(100);
        flag.HCSR04Flag=2;
        delay(2000);
        flag.HCSR04Flag=3;
        delay(2000);
        flag.HCSR04Flag=1;
        if ((distanceR >= distanceL) ) 
        {
          Left();
          delay(700);
          motors.stop();
          delay(200);
          flag.HCSR04Flag=1;
        } 
        else 
        { 
          Right();
          delay(700);
          motors.stop();
          delay(200);
          flag.HCSR04Flag=1;
        }
        // myservo.write(90);
        delay(1000);
      } 
      else 
      {
        flag.HCSR04Flag=1;
        Forward();
        delay(100);
        motors.stop();
        delay(30);
      }      
    }
    vTaskDelay(1);
  }

}


void vFunctionTask(void *pvParameters)
{
  (void)pvParameters;

  Serial.print(F("FunctionTask at core:"));
  Serial.println(xPortGetCoreID());
  for (;;) // A Task shall never return or exit.
  {
    if(flag.HCSR04Flag==1)
    {
      currentMillis = millis();
      // myservo.write(90);
      Left();
      delay(1000);
      motors.stop();
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10); 
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distance= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        Serial.print("Ultrasonic sensor is shown distance:");
        Serial.print(distance);
        Serial.println("cm");
        Serial.print(distanceR-distanceL);
        Serial.println("cm");
      }
    }
    if(flag.HCSR04Flag==2)  //lookRight
    {
      Left();
      delay(1000);
      motors.stop();
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceR= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        //Serial.print("Ultrasonic sensor is shown distanceR:");
        //Serial.print(distanceR);
        //Serial.println("cm");
      }
    }
    if(flag.HCSR04Flag==3)  //lookLeft
    {
      // myservo.write(160);
      Right();
      delay(2000);
      motors.stop();
      digitalWrite(TRIGPIN_PIN, LOW);  
      delayMicroseconds(2);  
      digitalWrite(TRIGPIN_PIN, HIGH); 
      delayMicroseconds(10);  
      digitalWrite(TRIGPIN_PIN, LOW);  
      duration= pulseIn(ECHO_PIN, HIGH);
      distanceL= duration/29/2;
      if (duration==0) 
      {
        Serial.println("No pulse is from sensor");
      }
      else 
      {
        //Serial.print("Ultrasonic sensor is shown distanceL:");
        //Serial.print(distanceL);
        //Serial.println("cm");
      }
    }
    vTaskDelay(1);
  }
}