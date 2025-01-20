#include "VOneMqttClient.h"
#include <ESP32Servo.h>

//sensors and actuators ID
const char* ServoMotor = "3578f485-3ce6-4e01-b625-cb258d7c57a2";
const char* PIRSensor = "c7a6004c-ca4e-4f30-8e24-598d6da94be0";
const char* ultrasonicSensor = "e0c4f3d1-9302-4098-bc3d-88c6bb35ffff";

//define Pins of sensors and actuators 
const int pirPin = 14; //pir
const int servoPin = 17; //servo motor
const int trigPin = 26; //ultrasonic trig pin
const int echoPin = 25; //ultrasonic echo pin

//initialization
int pirState = LOW; 
long duration;
float distance;
bool servoEnabled = true; // Variable to track servo motor state
boolean detection = false;

Servo servo;
VOneMqttClient voneClient;

void setup_wifi() 
{
  delay(10);
  // Start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void triggerActuator_callback(const char* actuatorDeviceId, const char* actuatorCommand) 
{
  Serial.print("Main received callback : ");
  Serial.print(actuatorDeviceId);
  Serial.print(" : ");
  Serial.println(actuatorCommand);

  String errorMsg = "";

  JSONVar commandObject = JSON.parse(actuatorCommand); 
  JSONVar keys = commandObject.keys(); 

  //check if auto lid open is ON/OFF
  if (String(actuatorDeviceId) == ServoMotor) 
  {
    String key = "";
    int commandValue;
    bool value;
    for (int i = 0; i < keys.length(); i++) 
    {
      key = (const char* )keys[i];
      commandValue = (int)commandObject[keys[i]];

      if (commandValue == 1)
      {
        Serial.println("This loop gets command value as 1.");
        value = true;
      }
      else if (commandValue == 0)
      {
        Serial.println("This loop gets command value as 0.");
        value = false;
      }
    }
    Serial.print("Key : ");
    Serial.println(key.c_str());
    Serial.print("Value : ");
    Serial.println(commandValue);
    
   
    if (value == true) 
    {
      Serial.println("Auto Lid Open: ON");
      servoEnabled = true; // Enable servo functionality
      voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);
    } 

    else if (value == false)
    {
      Serial.println("Auto Lid Open: OFF");
      servo.write(0);
      servoEnabled = false; // Disable aervo functionality
      voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), true);
    }
  } 

  else 
  {
    Serial.print("No actuator found: ");
    Serial.println(actuatorDeviceId);
    errorMsg = "No actuator found";
    voneClient.publishActuatorStatusEvent(actuatorDeviceId, actuatorCommand, errorMsg.c_str(), false); // Publish actuator status
  }
}

void ultrasonic()
{
  delay(200);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  //Publish telemtry data
  voneClient.publishTelemetryData(ultrasonicSensor, "Distance", distance);
}

void setup() 
{
  setup_wifi();
  voneClient.setup();
  voneClient.registerActuatorCallback(triggerActuator_callback);

  Serial.begin(9600);
  servo.attach(servoPin);

  pinMode(pirPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  ultrasonic();

  servo.write(0);
  Serial.println("System Initialized. Waiting for motion...");
}

void loop() 
{
  if (!voneClient.connected()) {
    voneClient.reconnect();
    String errorMsg = "Sensor Fail";
    voneClient.publishDeviceStatusEvent(PIRSensor, true);
    voneClient.publishDeviceStatusEvent(ultrasonicSensor, true);
  }
  voneClient.loop();

  int motionDetected = digitalRead(pirPin);

  if (motionDetected == HIGH && pirState == LOW) 
  {
    Serial.println("Motion detected!");
    pirState = HIGH;

    // Publish telemetry data
    detection = true;
    voneClient.publishTelemetryData(PIRSensor, "Motion", detection);

    // Rotate servo to 180 degrees if enabled
    if (servoEnabled) 
    {
      Serial.println("Opening lid...");
      servo.write(180);
      delay(8000); // Allow some time for the servo

      // Return servo to 0 degrees
      Serial.println("Closing lid after 10s...");
      detection = false;
      voneClient.publishTelemetryData(PIRSensor, "Motion", detection);
      servo.write(0);
      delay(100); // Small delay to stabilize servo
    } 

    ultrasonic();
  } 
  
  else if (motionDetected == LOW && pirState == HIGH) 
  {
    // Reset PIR state when no motion is detected
    pirState = LOW;
    Serial.println("No motion detected.");

    detection = false;
    voneClient.publishTelemetryData(PIRSensor, "Motion", detection);

    ultrasonic();
  }
}
