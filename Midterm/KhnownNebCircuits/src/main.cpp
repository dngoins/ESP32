/*
Original
Embedded System Midterm for Classes CDA4630 and CDA6316
Based on ESP32 technology, NodeMCU-32S
Dr. Bassem Alhalabi
Florida Atlantic Univercity, Boca Raton
Update: 2023-0928

Updated: 2024.05.25
by: Dwight Goins, MCT,RD, Graduate Student

Video Accompanying Project:
https://youtu.be/ueXpcHeXfcc?si=wkf0O5Zj8lw5V81l

references:
https://www.instructables.com/ESP32-Mic-Testing-With-INMP441-and-DumbDisplay/
https://github.com/sheaivey/ESP32-AudioInI2S
https://www.youtube.com/watch?v=m8LwPNXqK9o
https://www.youtube.com/watch?v=ueXpcHeXfcc

https://learn.microsoft.com/en-us/azure/azure-web-pubsub/key-concepts?WT.mc_id=Portal-Microsoft_Azure_SignalR
https://learn.microsoft.com/en-us/azure/azure-web-pubsub/reference-functions-bindings?tabs=csharp
https://github.com/MicrosoftLearning/mslearn-ai-language/blob/main/Instructions/Exercises/03-language-understanding.md
https://learn.microsoft.com/en-us/azure/azure-web-pubsub/tutorial-serverless-notification?tabs=csharp-isolated-process


*/

#include "Arduino.h"
#include <ArduinoJson.h>


// WiFi
#include <WiFiClientSecure.h>
#include <HTTPClient.h>


// I2S driver
#include <driver/i2s.h>
#include <SPIFFS.h>

 // Web sockets
#include <WebSocketsClient.h>

// for helping generate WAV header
#include <cstdint>
#include <cstring>
#include <vector>

// Servo
#include <ESP32Servo.h>


// INMP441 I2S pin assignment
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_SAMPLE_BIT_COUNT 16
#define SOUND_SAMPLE_RATE 8000
#define SOUND_CHANNEL_COUNT 1
#define I2S_PORT I2S_NUM_0
#define CHUNK_SIZE 8

File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;

// In your
 // Function to extract the path from a URL
const char* extractPathFromUrl(const char* url);
void websocketConnect();
void toggle();
void processVoiceCommand(const char* command) ;
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void connect();
void toggleVoice(bool turnOn = true);
void lock();
void unlock();
void turnOnFan();
void turnOffFan();
void turnOnLights(int ledPin);
void turnOffLights(int ledPin);
void turnUpHeat();
void turnDownHeat();
void turnUpAC();
void turnDownAC();
void turnUpSpeaker();
void turnDownSpeaker();
// // I2S functions
esp_err_t i2s_install();
esp_err_t i2s_setpin();
void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len);


 
// class obects
Servo myservo;  // using class servo to create servo object, myservo, to control a servo motor
// Download the library from: https://github.com/jkb-git/ESP32Servo (Press on Code, and download as zip).
// Unzip and rename the folder to ESP32Servo.
// Put the folder in Document>Arduino>libraries


//TODO: Replace with your network credentials
// Replace with your network credentials
const char* WIFI_SSID     = "wifi_ssid";
const char* WIFI_PASS = "wifi_password";

WiFiClientSecure client;


// Define the Azure Speech to Text Web Socket
WebSocketsClient webSocket;

//TODO: Replace with your Azure Function endpoint
// http://199.199.1.155:7136
// Deploy your own Azure Function and replace with your endpoint
const char* negotiate_url = "https://YourAzureEndpoing/api/negotiate"; 

HTTPClient http;

 bool useVoice = false;

// name of recorded WAV file; since only a single name; hence new one will always overwrite old one
const char* SoundName = "recorded_sound";

const int I2S_DMA_BUF_COUNT = 10;
const int I2S_DMA_BUF_LEN = 1024;

// ---------- defining pins for outputs
const int greenLightLed = 18;   // green led
const int blueLightLed = 21;    // ligh blue led
const int yellowLightLed = 22;  // light yellow led - pwm
const int redLightLed = 23;     // temperature red led
const int dcMotorPin = 19;

// ---------- defining pins for inputs
const int servoPin = 3;  // servo pin
//const int distPin = 35;         // ir analog pin
const int lightSensorPin = 36;  // light sensor pins
const int pirSensorPin = 34;    // touch sensor pin
const int tempSensorPin = 39;   // temperature sensor pin

bool ledState = false; // used toggle Lights

// micConfiguration
const int digitalMicPin = 34;
const int analogMicPin = 34;

int lastSoundState = HIGH;  // the previous state from the input pin
double currentSoundState;   // the current reading from the input pin

#if I2S_SAMPLE_BIT_COUNT == 32
const int StreamBufferNumBytes = 512;
const int StreamBufferLen = StreamBufferNumBytes / 4;
int32_t StreamBuffer[StreamBufferLen];
#else
#if SOUND_SAMPLE_RATE == 16000
  // for 16 bits ... 16000 sample per second (32000 bytes per second; since 16 bits per sample) ==> 512 bytes = 16 ms per read
const int StreamBufferNumBytes = 512;
#else
  // for 16 bits ... 8000 sample per second (16000 bytes per second; since 16 bits per sample) ==> 256 bytes = 16 ms per read
const int StreamBufferNumBytes = 256;
#endif
const int StreamBufferLen = StreamBufferNumBytes / 2;
int16_t StreamBuffer[StreamBufferLen];
#endif

// sound sample (16 bits) amplification
const int MaxAmplifyFactor = 20;
const int DefAmplifyFactor = 10;


int amplifyFactor = DefAmplifyFactor;  //10;
bool isWSSConnected = false;

int baseTempVal = 0;

// ---------- Main functions 

// trim from start (in place)
inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

void processVoiceCommand(const char* command) {
  Serial.print("Command received: ");
  Serial.println(command);
  std::string cmd(command);
  ltrim(cmd);
  rtrim(cmd);

  if (cmd == "[iot] reset")
  {
    ESP.restart();
  }
  
  if (cmd == "[iot] toggle") {
    toggle();  
    return;
  }

   if (cmd == "[iot] toggleVoice") {
    toggleVoice(false);  
    return;

  }

  if (cmd.find("[iot] unlock") != std::string::npos) {
    unlock();  
    return;

  }
    
  if (cmd.find("[iot] lock") != std::string::npos)  {
    lock();  
    return;

  }

  if (cmd.find("turnOn fan") != std::string::npos) {
    turnOnFan(); 
    return;

  }
  
  if (cmd.find("turnOff fan") != std::string::npos) {
    turnOffFan();  
    return;

  }


  if (cmd.find( "turnOn kitchen") != std::string::npos) {
    turnOnLights(blueLightLed);  
    delay(2000);
    return;

  }
  
  if (cmd.find("turnOff kitchen") != std::string::npos) {
    turnOffLights(blueLightLed); 
    return;

  }

  if (cmd.find( "turnOn blue") != std::string::npos) {
    turnOnLights(blueLightLed);  
    delay(2000);
    return;

  }
  
  if (cmd.find("turnOff blue") != std::string::npos) {
    turnOffLights(blueLightLed); 
    return;

  }

if (cmd.find( "turnOn lights") != std::string::npos) {
    turnOnLights(0);  
    return;

  }
  
  if (cmd.find("turnOff lights") != std::string::npos) {
    turnOffLights(0); 
    return;

  }

  if (cmd.find( "turnOn bathroom") != std::string::npos) {
    turnOnLights(redLightLed);  
    delay(2000);
    return;

  }
  
  if (cmd.find("turnOff bathroom") != std::string::npos) {
    turnOffLights(redLightLed); 
    return;

  }

 if (cmd.find( "turnOn bedroom") != std::string::npos) {
    turnOnLights(redLightLed);  
    delay(2000);
    return;

  }
  
  if (cmd.find("turnOff bedroom") != std::string::npos) {
    turnOffLights(redLightLed);   
    return;

  }
   if (cmd.find( "turnOn red") != std::string::npos) {
    turnOnLights(redLightLed);  
    delay(2000);
    return;

  }
  
  if (cmd.find("turnOff red") != std::string::npos) {
    turnOffLights(redLightLed);   
    return;

  }

if (cmd.find( "turnOn patio") != std::string::npos) {
    turnOnLights(yellowLightLed); 
    return;

  }
  
  if (cmd.find("turnOff patio") != std::string::npos) {
    turnOffLights(yellowLightLed); 
    return;

  }

  if (cmd.find( "turnOn yellow") != std::string::npos) {
    turnOnLights(yellowLightLed); 
    return;

  }
  
  if (cmd.find("turnOff yellow") != std::string::npos) {
    turnOffLights(yellowLightLed); 
    return;

  }

if (cmd.find( "turnOn garage") != std::string::npos) {
    turnOnLights(greenLightLed);  
    return;

  }
  
  if (cmd.find("turnOff garage") != std::string::npos) {
    turnOffLights(greenLightLed); 
    return;

  }

  if (cmd.find( "turnOn green") != std::string::npos) {
    turnOnLights(greenLightLed);  
    return;

  }
  
  if (cmd.find("turnOff green") != std::string::npos) {
    turnOffLights(greenLightLed); 
    return;

  }


  if (cmd.find("turnUp heat") != std::string::npos) {
    turnUpHeat(); 
    return;

  }
  
  if (cmd.find( "turnDown heat") != std::string::npos) {
    turnDownHeat(); 
    return;

  }


  if (cmd.find( "turnUp ac") != std::string::npos) {
    turnUpAC();  
    return;

  }
  
  if (cmd.find("turnDown ac") != std::string::npos) {
    turnDownAC();  
    return;

  }

  if (cmd.find( "turnUp Speaker") != std::string::npos) {
    turnUpSpeaker(); 
    return;

  }
  
  if (cmd.find("turnDown Speaker") != std::string::npos) {
    turnDownSpeaker();  
    return;

  }
  
    for (int i = 0; i < 3; i++) {
      turnOnLights(redLightLed);
      delay(100);
      turnOnLights(blueLightLed);
      delay(100);
      turnOnLights(yellowLightLed);
      delay(100);
      turnOnLights(greenLightLed);
      delay(300);
      turnOffLights(0);      
    }
    Serial.print(cmd.c_str());
    Serial.println(" - Command not recognized");
}

void lock()
{
  Serial.println("locking - turning servo motor");
  myservo.write(180);
}

void unlock()
{
  Serial.println("unlocking - turning servo motor");
  myservo.write(0);
}

void turnOnFan()
{
  Serial.println("turnOnFan");
  digitalWrite(dcMotorPin, HIGH);
}

void turnOffFan()
{
  Serial.println("turnOffFan");
  digitalWrite(dcMotorPin, LOW);
}

void turnOnLights(int ledPin = 0)
{
  if (ledPin == 0)
  {
   
  //Serial.println("turnOnLights");
  digitalWrite(blueLightLed, HIGH );
  digitalWrite(redLightLed, HIGH );
  digitalWrite(greenLightLed,  HIGH );
  digitalWrite(yellowLightLed,  HIGH ); 
  }
  else
  {
  //  Serial.println("turnOnLights");
    digitalWrite(ledPin, HIGH );
  }
}

void turnOffLights(int ledPin = 0)
{
 // Serial.println("turnOffLights");
  if(ledPin == 0)
  {
    digitalWrite(blueLightLed, LOW);
    digitalWrite(redLightLed,  LOW);
    digitalWrite(greenLightLed,  LOW);
    digitalWrite(yellowLightLed,  LOW); 
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
}

void turnUpHeat()
{
  Serial.println("turnUp Heat Changing Thermostat");
  baseTempVal = baseTempVal - 5;
}

void turnDownHeat()
{
  Serial.println("turnDown Heat Changing Thermostat");
  baseTempVal = baseTempVal + 5;
}

void turnUpAC()
{
  Serial.println("turnUp AC putting on DC Fan ");
  baseTempVal = baseTempVal + 5;
}

void turnDownAC()
{
  Serial.println("turnDownAC");
  baseTempVal = baseTempVal - 5;
}

void turnUpSpeaker()
{
  Serial.println("turnUpSpeker");
}

void turnDownSpeaker()
{
  Serial.println("turnDownSpeaker");
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {

    Serial.print("Socket Event Type: "); Serial.print(type);
    Serial.print(" Payload Length: "); Serial.print(length);
    Serial.print(" Payload: "); Serial.println((const char*)payload);

    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket disconnected...");
            isWSSConnected = false;
           // websocketConnect();
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket connected");
            isWSSConnected = true;
            break;        
        case WStype_TEXT:
            // Handle incoming WebSocket data (payload)          
            processVoiceCommand((const char*)payload);
            break;
        // Add other cases as needed (PING, PONG, etc.)
        case WStype_ERROR:
            Serial.println("WebSocket error");
            break;
        case WStype_BIN:            
        case WStype_FRAGMENT_TEXT_START:            
        case WStype_FRAGMENT_BIN_START:                        
        case WStype_FRAGMENT:             
        case WStype_FRAGMENT_FIN: 
        case WStype_PING:
        case WStype_PONG:
        default:
            break;
        
    }
}

void connect() {

  // Connect to Wifi.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("WiFi - Connected!");
  Serial.println("This device is now ready for use!");
}

void websocketConnect()
{
   if (WiFi.status() == WL_CONNECTED) {
       HTTPClient http;
       http.begin(negotiate_url);
       
       int httpCode = http.GET();
       Serial.print("HTTP CODE: "); Serial.println(httpCode);

       if (httpCode == HTTP_CODE_OK) {
            String response = http.getString(); 
       
           // Extract the url from the response
            DynamicJsonDocument doc(1024);
            deserializeJson(doc, response);
            const char* url = doc["url"];
            Serial.print("URL: "); Serial.println(url);

            // Create a mutable copy of the URL
            char urlCopy[strlen(url) + 1];
            strcpy(urlCopy, url);

            // Extract the hostname using strtok
            // now extract the host name from the dynamic url
            
            strtok(urlCopy, "/");           
            char* fakehostName2 = strtok(NULL, "/");
            const char* hostName = strtok(fakehostName2, ":");

            Serial.print("Host Name: "); Serial.println(hostName);
            
            // extract the port number as int from the dynamic url
            const char* portStr = strtok(NULL, ":");
            //const char* portStr = strtok(NULL, "/");
            int port = atoi(portStr);            
            Serial.print("Port: "); Serial.println(port);

            // extract the path in the url and extract all content after that and store it in path variable
            const char* _path = extractPathFromUrl(url);
            const char * path = extractPathFromUrl(_path);

            Serial.print("Path: "); Serial.println(path);
            
            // Initialize Websocket
            // Setup WebSocket event Handlers
            webSocket.onEvent(webSocketEvent);

            // Connect to the WebSocket server
            webSocket.beginSSL(hostName, port, path);

            // try ever 5000 again if connection has failed
            webSocket.setReconnectInterval(5000);

         }
    }
  
}


void toggle() {
  Serial.println("Toggling LED.");
  ledState = !ledState;
  digitalWrite(blueLightLed, ledState ? HIGH : LOW);
  digitalWrite(redLightLed, ledState ? HIGH : LOW);
  digitalWrite(greenLightLed, ledState ? HIGH : LOW);
  digitalWrite(yellowLightLed, ledState ? HIGH : LOW);
  useVoice = !useVoice;  
}

void toggleVoice(bool turnOn)
{
  if (turnOn) 
    useVoice = true;
  else
    useVoice = !useVoice;

  if(useVoice == false)
  {
    Serial.println("Turning off Voice.");
    if(isWSSConnected)
      webSocket.sendTXT("stop");
  }
  else
    Serial.println("Turning on Voice.");
  
}

// setup function: init all connections
void setup() 
{ 
  pinMode(lightSensorPin, INPUT);
  pinMode(pirSensorPin, INPUT);
  pinMode(tempSensorPin, INPUT);
  pinMode(digitalMicPin, INPUT);
  // pinMode(analogMicPin, INPUT);


  // // set output pins
  pinMode(greenLightLed, OUTPUT);
  pinMode(blueLightLed, OUTPUT);
  pinMode(yellowLightLed, OUTPUT);
  pinMode(redLightLed, OUTPUT);
  pinMode(dcMotorPin, OUTPUT);
  

  // // set up default values
  digitalWrite(greenLightLed, HIGH);
  digitalWrite(blueLightLed, HIGH);
  digitalWrite(yellowLightLed, HIGH);
  digitalWrite(redLightLed, HIGH);
  digitalWrite(dcMotorPin, HIGH);
  delay(1000);  // wait for 1000 ms

  myservo.attach(servoPin);
  myservo.write (180);
  delay(1000);  // wait for 1000 ms
  myservo.write (0);
  delay(1000);  // wait for 1000 ms

  digitalWrite(greenLightLed, LOW);
  digitalWrite(blueLightLed, LOW);
  digitalWrite(yellowLightLed, LOW);
  digitalWrite(redLightLed, LOW);
  digitalWrite(dcMotorPin, LOW);
  delay(1000);  // wait for 1000 ms

  // start the serial port
  Serial.begin(115200);

  
  connect();
  websocketConnect();

  Serial.println("SETUP MIC ...");

  // set up I2S
  if (i2s_install() != ESP_OK) {
    Serial.println("XXX failed to install I2S");
  }
  if (i2s_setpin() != ESP_OK) {
    Serial.println("XXX failed to set I2S pins");
  }
  if (i2s_zero_dma_buffer(I2S_PORT) != ESP_OK) {
    Serial.println("XXX failed to zero I2S DMA buffer");
  }
  if (i2s_start(I2S_PORT) != ESP_OK) {
    Serial.println("XXX failed to start I2S");
  }
  
  Serial.println("... DONE SETUP MIC");
 
  baseTempVal =  analogRead(tempSensorPin);   // room temperature

}


void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
}


void loop() 
{

  // wifi
  bool toReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }


  if (toReconnect) 
  {
    connect();
  }

  webSocket.loop();

  auto tempVal = analogRead(tempSensorPin);
    // Just choose a 4% - 8% range to determine when to turn the Red light on
  // so it doesn't flip back and forth on specific temerature
  if (tempVal > 1.08 * baseTempVal) { digitalWrite(redLightLed, HIGH);  } 
  if (tempVal < 1.04 * baseTempVal) { digitalWrite(redLightLed, LOW);   }
  

   // Read the microphone Sound state
  currentSoundState = digitalRead(digitalMicPin) ;
  
  //Serial.print("Digital Read: "); Serial.println(digitalMicPin);

  // float voltage = currentSoundState *  (5.0 / 1023.0); 
  
   if (currentSoundState == HIGH)
    {
      digitalWrite(blueLightLed, HIGH);
      Serial.println("Loud Noise heard, turning on the microphone to listen, say Toggle Voice to turn off");
      toggleVoice(true);
      delay(1000);
    }
    else
    {
       digitalWrite(blueLightLed, LOW);
    }

//   // read I2S data and place in data buffer
   size_t bytesRead = 0;
   esp_err_t result = i2s_read(I2S_PORT, &StreamBuffer, StreamBufferNumBytes, &bytesRead, portMAX_DELAY);
   i2s_adc_data_scale((uint8_t*)StreamBuffer, (uint8_t*)StreamBuffer, bytesRead);

    if (useVoice) {
      webSocket.sendBIN((uint8_t*)StreamBuffer, bytesRead);
    }

}


esp_err_t i2s_install() {
  uint32_t mode = I2S_MODE_MASTER | I2S_MODE_RX;
#if I2S_SCK == I2S_PIN_NO_CHANGE
    mode |= I2S_MODE_PDM;
#endif    
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(mode/*I2S_MODE_MASTER | I2S_MODE_RX*/),
    .sample_rate = SOUND_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BIT_COUNT),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = I2S_DMA_BUF_COUNT/*4*/,
    .dma_buf_len = I2S_DMA_BUF_LEN/*1024*/,
    .use_apll = false
  };
  return i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
 
esp_err_t i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,   
    .data_out_num = I2S_PIN_NO_CHANGE/*-1*/,
    .data_in_num = I2S_SD
  };
  return i2s_set_pin(I2S_PORT, &pin_config);
}

// Function to extract the path from a URL
const char* extractPathFromUrl(const char* url) {
    
    // Find the position of ":"
    const char* portStart = strstr(url, ":");
    if (!portStart) {
      return nullptr;
    }

    // Find the position of "/"
    const char* pathStart = strstr(portStart, "/");
    if (!pathStart) {
        return nullptr;
    }
    
    // Extract the path (including query string)
    return pathStart;
}