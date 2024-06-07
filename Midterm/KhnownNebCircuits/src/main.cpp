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

*/

#include "Arduino.h"
#include <ArduinoJson.h>


// WiFi
#include <WiFiClientSecure.h>
#include <HTTPClient.h>


// I2S driver
#include <driver/i2s.h>


 // Web sockets
#include <WebSocketsClient.h>

// INMP441 I2S pin assignment
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define I2S_SAMPLE_BIT_COUNT 16
#define SOUND_SAMPLE_RATE 16000
#define SOUND_CHANNEL_COUNT 1
#define I2S_PORT I2S_NUM_0
#define CHUNK_SIZE 8
#define I2S_MODE I2S_MODE_RX

// if want Bluetooth, uncomment the following line
#define BLUETOOTH "ESP32BT"
#if defined(BLUETOOTH)
//#include "esp32dumbdisplay.h"
//DumbDisplay dumbdisplay(new DDBluetoothSerialIO(BLUETOOTH));
#else
#include "wifidumbdisplay.h"
DumbDisplay dumbdisplay(new DDWiFiServerIO(WIFI_SSID, WIFI_PASSWORD));
#endif

// // Dumb Display
// PlotterDDLayer* plotterLayer;
// LcdDDLayer* micTabLayer;
// LcdDDLayer* recTabLayer;
// LcdDDLayer* playTabLayer;
// LcdDDLayer* startBtnLayer;
// LcdDDLayer* stopBtnLayer;
// LcdDDLayer* amplifyLblLayer;
// LedGridDDLayer* amplifyMeterLayer;

// In your
 // Function to extract the path from a URL
const char* extractPathFromUrl(const char* url);
void websocketConnect();
void toggle();
void processVoiceCommand(const char* command) ;
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);
void connect();
void toggleVoice();
void lock();
void unlock();
void turnOnFan();
void turnOffFan();
void turnOnLights();
void turnOffLights();
void turnUpHeat();
void turnDownHeat();
void turnUpAC();
void turnDownAC();
void turnUpSpeaker();
void turnDownSpeaker();
// // I2S functions
esp_err_t i2s_install();
esp_err_t i2s_setpin();
   
 
// class obects
//Servo myservo;  // using class servo to create servo object, myservo, to control a servo motor
// Download the library from: https://github.com/jkb-git/ESP32Servo (Press on Code, and download as zip).
// Unzip and rename the folder to ESP32Servo.
// Put the folder in Document>Arduino>libraries

// Replace with your network credentials
const char* WIFI_SSID     = "home";
const char* WIFI_PASS = "888@999.000;";

WiFiClientSecure client;

// Include the Web Socket library

// Define the Azure Speech to Text Web Socket
WebSocketsClient webSocket;

// http://199.199.1.155:7136
const char* negotiate_url = "https://knowncircuits.azurewebsites.net"; \\ Deploy your own Azure Function and replace with your endpoint

HTTPClient http;

 bool useVoice = false;

// name of recorded WAV file; since only a single name; hence new one will always overwrite old one
const char* SoundName = "recorded_sound";

const int I2S_DMA_BUF_COUNT = 8;
const int I2S_DMA_BUF_LEN = 1024;

// ---------- defining pins for outputs
const int greenLightLed = 18;   // green led
const int blueLightLed = 21;    // ligh blue led
const int yellowLightLed = 22;  // light yellow led - pwm
const int redLightLed = 23;     // temperature red led
const int dcMotorPin = 19;

// ---------- defining pins for inputs
const int servoPin = 35;  // servo pin
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

//DDConnectVersionTracker cvTracker;  // it is for tracking [new] DD connection established
int what = 1;                       // 1: mic; 2: record; 3: play
bool started = false;
int amplifyFactor = DefAmplifyFactor;  //10;
int soundChunkId = -1;                 // when started sending sound [chunk], the allocated "chunk id"
long streamingMillis = 0;
int streamingTotalSampleCount = 0;

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

  
  if (cmd == "[iot] toggle") {
    toggle();  
  }

   if (cmd == "[iot] toggleVoice") {
    toggleVoice();  
  }

  if (cmd.find("unlock") != std::string::npos) {
    unlock();  
  }
    
  if (cmd.find("lock") != std::string::npos)  {
    lock();  
  }

  if (cmd.find("turnOn Fan") != std::string::npos) {
    turnOnFan();  
  }
  
  if (cmd.find("turnOff Fan") != std::string::npos) {
    turnOffFan();  
  }


  if (cmd.find( "turnOn Lights") != std::string::npos) {
    turnOnLights();  
  }
  
  if (cmd.find("turnOff Lights") != std::string::npos) {
    turnOffLights();  
  }


  if (cmd.find("turnUp Heat") != std::string::npos) {
    turnUpHeat();  
  }
  
  if (cmd.find( "turnDown Heat") != std::string::npos) {
    turnDownHeat();  
  }


  if (cmd.find( "turnUp AC") != std::string::npos) {
    turnUpAC();  
  }
  
  if (cmd.find("turnDown AC") != std::string::npos) {
    turnDownAC();  
  }

  if (cmd.find( "turnUp Speaker") != std::string::npos) {
    turnUpSpeaker();  
  }
  
  if (cmd.find("turnDown Speaker") != std::string::npos) {
    turnDownSpeaker();  
  }
}

void lock()
{
  Serial.println("locking - turning servo motor");
}

void unlock()
{
  Serial.println("unlocking - turning servo motor");
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

void turnOnLights()
{
  Serial.println("turnOnLights");
  digitalWrite(blueLightLed, HIGH );
  digitalWrite(redLightLed, HIGH );
  digitalWrite(greenLightLed,  HIGH );
  digitalWrite(yellowLightLed,  HIGH ); 
}

void turnOffLights()
{
  Serial.println("turnOffLights");
  
  digitalWrite(blueLightLed, LOW);
  digitalWrite(redLightLed,  LOW);
  digitalWrite(greenLightLed,  LOW);
  digitalWrite(yellowLightLed,  LOW); 

}

void turnUpHeat()
{
  Serial.println("turnUp Heat Changing Thermostat");
}

void turnDownHeat()
{
  Serial.println("turnDown Heat Changing Thermostat");
}

void turnUpAC()
{
  Serial.println("turnUp AC putting on DC Fan ");
}

void turnDownAC()
{
  Serial.println("turnDownAC");
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
           // websocketConnect();
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket connected");
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
}

void toggleVoice()
{
  Serial.println("Toggling Voice.");
  useVoice = !useVoice;  
}

// setup function: init all connections
void setup() 
{ 
  pinMode(servoPin, INPUT);
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

  // myservo.write (180);
  // delay(1000);  // wait for 1000 ms
  // myservo.write (0);
  // delay(1000);  // wait for 1000 ms

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

  
}


// generic function to return the average value in the collection
// of sampled data points
int calcAvgValues(const int size, const int samples[])
{
  if (size == 0) return samples[0];

  long totals = 0;
  for(int i = 0; i < size; i++)
  {
    totals += samples[i];  
  }

  return (int)(totals/size);
}


void loop() 
{

  // wifi
  bool toReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Disconnected from WiFi");
    toReconnect = true;
  }

  // if (!device.connected()) {
  //   Serial.println("Disconnected from MQTT");
  //   Serial.println(device.mqttClient.state());
  //   toReconnect = true;
  // }

  if (toReconnect) 
  {
    connect();
  }

  webSocket.loop();

   // Read the microphone Sound state
  currentSoundState = digitalRead(digitalMicPin) ;
  
  //Serial.print("Digital Read: "); Serial.println(digitalMicPin);

  // float voltage = currentSoundState *  (5.0 / 1023.0); 
  
   if (currentSoundState == HIGH)
    {
      digitalWrite(blueLightLed, HIGH);
      delay(1000);
    }
    else
    {
       digitalWrite(blueLightLed, LOW);
    }

  // read I2S data and place in data buffer
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, &StreamBuffer, StreamBufferNumBytes, &bytesRead, portMAX_DELAY);

  int samplesRead = 0;
#if I2S_SAMPLE_BIT_COUNT == 32
  int16_t sampleStreamBuffer[StreamBufferLen];
#else
  int16_t* sampleStreamBuffer = StreamBuffer;
#endif
  if (result == ESP_OK) {
#if I2S_SAMPLE_BIT_COUNT == 32
    samplesRead = bytesRead / 4;  // 32 bit per sample
#else
    samplesRead = bytesRead / 2;  // 16 bit per sample
#endif
    if (samplesRead > 0) {
      // find the samples mean ... and amplify the sound sample, by simply multiple it by some "amplify factor"
      float sumVal = 0;
      for (int i = 0; i < samplesRead; ++i) {
        int32_t val = StreamBuffer[i];
#if I2S_SAMPLE_BIT_COUNT == 32
        val = val / 0x0000ffff;
#endif
        if (amplifyFactor > 1) {
          val = amplifyFactor * val;
          if (val > 32700) {
            val = 32700;
          } else if (val < -32700) {
            val = -32700;
          }
          //StreamBuffer[i] = val;
        }
        sampleStreamBuffer[i] = val;
        sumVal += val;
      }
      float meanVal = sumVal / samplesRead;
     //Serial.print("Mean Value: "); Serial.println(meanVal);
    }
  }

 if (useVoice)
 {
    
        // Calculate the number of bytes per chunk
          int bytesPerChunk = 320 * 2;  // 320 samples/chunk * 2 bytes/sample (16 bit = 2 bytes)

          // Send the audio data in chunks
          for (int i = 0; i < bytesRead; i += bytesPerChunk) {
            webSocket.sendBIN((uint8_t *)(StreamBuffer + i), bytesPerChunk);
          }
      
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
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT/*4*/,
    .dma_buf_len = I2S_DMA_BUF_LEN/*1024*/,
    .use_apll = true
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