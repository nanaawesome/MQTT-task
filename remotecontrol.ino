/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
  edited code for remote control with EEEbot
*********/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#define Rad2Deg  180 / M_PI;
const int MPU = 0x68; // MPU6050 I2C address
float Roll_r, Pitch_r, Yaw_r;
float AccX, AccY, AccZ,GyroX,GyroZ,GyroY;
float AngleRoll, AnglePitch, AngleSide ;



// Replace the next variables with your SSID/Password combination
const char* ssid = "harshalRaspi";
const char* password = "harshalRaspiraspi";                

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
// comand on Raspi : 
const char* mqtt_server = "192.168.2.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//uncomment the following lines if you're using SPI
/*#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5*/

void sendData(int16_t leftMotorspeed, int16_t rightMotorspeed, int16_t servoAngle) {

  Wire.beginTransmission(4); // Call slave

  Wire.write((byte)((leftMotorspeed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotorspeed & 0x000000FF));           // second byte of leftMotor_speed, containing bits 8 to 1
  Wire.write((byte)((rightMotorspeed & 0x0000FF00) >> 8));   // rest follows the same logic
  Wire.write((byte)(rightMotorspeed & 0x000000FF));          
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         
  Wire.write((byte)(servoAngle & 0x000000FF));               
  Wire.endTransmission();   // stop transmitting
}

void acc() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Register for low-passfilter
  Wire.write(0x05);  // Bandwidth 10Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // register for adjusting scale
  Wire.write(0x03); // +/- 16g Full Scale Range
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX=(AccXLSB/2048) ;
  AccY=(AccYLSB/2048) ;
  AccZ=(AccZLSB/2048) ;
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) * Rad2Deg;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ)) * Rad2Deg;
}

// LED Pin
//const int ledPin = 4;

void setup() {
  Serial.begin(115200);
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  //status = bme.begin();  
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  //pinMode(ledPin, OUTPUT);

}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  /*
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
  */
  if (String(topic) == "esp32/forward") {
  Serial.print("Changing output to ");
  if(messageTemp == "on"){
    Serial.println("on");
    int16_t leftMotorspeed = 200;
    int16_t rightMotorspeed = 200;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
  else if(messageTemp == "off"){
    Serial.println("off");
    int16_t leftMotorspeed = 0;
    int16_t rightMotorspeed = 0;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
}

  if (String(topic) == "esp32/backwards") {
  Serial.print("Changing output to ");
  if(messageTemp == "on"){
    Serial.println("on");
    int16_t leftMotorspeed = -200;
    int16_t rightMotorspeed = -200;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
  else if(messageTemp == "off"){
    Serial.println("off");
    int16_t leftMotorspeed = 0;
    int16_t rightMotorspeed = 0;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
}

  if (String(topic) == "esp32/left") {
  Serial.print("Changing output to ");
  if(messageTemp == "on"){
    Serial.println("on");
    int16_t leftMotorspeed = 200;
    int16_t rightMotorspeed = 200;
    int16_t servoAngle = 0;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
  else if(messageTemp == "off"){
    Serial.println("off");
    int16_t leftMotorspeed = 0;
    int16_t rightMotorspeed = 0;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
}

  if (String(topic) == "esp32/right") {
  Serial.print("Changing output to ");
  if(messageTemp == "on"){
    Serial.println("on");
    int16_t leftMotorspeed = 200;
    int16_t rightMotorspeed = 200;
    int16_t servoAngle = 180;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
  else if(messageTemp == "off"){
    Serial.println("off");
    int16_t leftMotorspeed = 0;
    int16_t rightMotorspeed = 0;
    int16_t servoAngle = 125;
    sendData(leftMotorspeed, rightMotorspeed, servoAngle);
  }
}
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/forward");
      client.subscribe("esp32/backwards");
      client.subscribe("esp32/left");
      client.subscribe("esp32/right");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 50) {
    lastMsg = now;
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
    float angle =  atan2(AccX,AccZ) * (180 / 3.14);
    Serial.println(angle);
    // Temperature in Celsius
    int temperature = angle ;   
    // Uncomment the next line to set temperature in Fahrenheit 
    // (and comment the previous temperature line)
    //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
    
    // Convert the value to a char array
    char tempString[8];
    dtostrf(temperature, 1, 2, tempString);
    Serial.print("Temperature: ");
    Serial.println(tempString);
    client.publish("esp32/temperature", tempString);


    Wire.requestFrom(4, 2); // request 2 bytes from slave device with address 8
    while (Wire.available()) {
    uint8_t enc1 = Wire.read();
    uint8_t enc1_2 = Wire.read();
    uint8_t enc2 = Wire.read();
    uint8_t enc2_2 = Wire.read();

    uint16_t enc1_count = (enc1 << 8)|enc1_2;
    uint16_t enc2_count = (enc2 << 8)|enc2_2;
    
        
    char temp1[16];
    dtostrf(enc1_count, 1, 2, temp1);
    Serial.print("Temperature: ");
    Serial.println(temp1);
    client.publish("esp32/enc1", temp1);

    char temp2[16];
    dtostrf(enc2_count, 1, 2, temp2);
    Serial.print("Temperature: ");
    Serial.println(temp2);
    client.publish("esp32/enc2", temp2);
    
  }
 }
}

// send instructions to EEEbot


