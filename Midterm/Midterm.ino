/*
Original
Embedded System Midterm for Classes CDA4630 and CDA6316
Based on ESP32 technology, NodeMCU-32S
Dr. Bassem Alhalabi
Florida Atlantic Univercity, Boca Raton
Update: 2023-0928

Updated: 2024.05.25
by: Dwight Goins, MCT,RD, Graduate Student
*/

// include files
#include <ESP32_Servo.h>

// class obects
Servo myservo;  // using class servo to create servo object, myservo, to control a servo motor
// Download the library from: https://github.com/jkb-git/ESP32Servo (Press on Code, and download as zip).
// Unzip and rename the folder to ESP32Servo.
// Put the folder in Document>Arduino>libraries


// ---------- defining pins for inputs 
const int servoPin = 23;      // servo pin
const int distPin = 35;         // ir analog pin
const int lightPin = 36;      // light sensor pins
const int touchPin = 34;      // touch sensor pin
const int tempPin = 39;       // temperature sensor pin

// ---------- defining pins for outputs 
const int touchLed = 25;      // touch green led
const int lightLed = 32;      // ligh blue led
const int lightPwmLed = 22;   // light yellow led - pwm
const int tempLed = 33;       // temperature red led
const int BUFFER_SIZE = 10;

//  ---------- sensor values
int touchVal = 0, baseTouchVal=0;
int lightVal = 0, baseLightVal = 0, pwmVal=0, mapPwmVal=0;
int tempVal = 0, baseTempVal = 0;
int distVal = 0, baseDistVal=0, mapDistVal =0;
int flag=0;
int lastPos = 0;
bool handQuick = false;
int sampleIndex = 0;
int roomLightReadings[BUFFER_SIZE];
int touchReadings[BUFFER_SIZE];

bool toggleTouch = false;

// ---------- Main functions 

// setup function: init all connections
void setup() 
{
  // Set up ADC resolution to 10 bits: will get values between 0-1024
  analogReadResolution(10);  
  // connect servo myservo to servoPin
  myservo.attach(servoPin);
  
  // set up analog pin as inputs
  pinMode(distPin, INPUT);
  pinMode(lightPin, INPUT);
  pinMode(touchPin, INPUT);
  pinMode(tempPin, INPUT);

  // set output pins
  pinMode(touchLed, OUTPUT);
  pinMode(lightLed, OUTPUT);
  pinMode(tempLed, OUTPUT);
  pinMode(lightPwmLed, OUTPUT);
 
  // set up default values
  digitalWrite(touchLed, HIGH);
  digitalWrite(lightLed, HIGH);
  digitalWrite(tempLed, HIGH);
  digitalWrite(lightPwmLed, HIGH);
  delay(1000);  // wait for 1000 ms

  myservo.write (180);
  delay(1000);  // wait for 1000 ms
  myservo.write (0);
  delay(1000);  // wait for 1000 ms

  digitalWrite(touchLed, LOW);
  digitalWrite(lightLed, LOW);
  digitalWrite(tempLed, LOW);
  digitalWrite(lightPwmLed, LOW);
  delay(1000);  // wait for 1000 ms

  // setup Data Sample Buffer

  // start the serial port
  Serial.begin(115200);

  // read sensors values and save them as baseline values
  baseLightVal = analogRead(lightPin);  // room light
  baseTempVal =  analogRead(tempPin);   // room temperature
  baseTouchVal = analogRead(touchPin);  // room touch
  baseDistVal = analogRead(distPin);  // room touch
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

  // ---------- distance sensor controls
  // read the distance sensor, set the min/max limits, and map to full range 0-180 
  distVal = analogRead(distPin);  // 0 - 1023
  
  distVal = min (distVal, 1000);
  distVal = max (distVal, 250);  
  mapDistVal = map (distVal, 250, 1000, 0, 180);  // map values between in range (250, 1010) to values in range (0, 180)
  
  // ********** requirement # 1
  // the code above cuases the servo to sweep from 0-90 as the distVal sweeps from 250-1000 as you vertically move your hand up/down over the distance sensor
  // modify the code so that if you move your hand horizantally quickly the servo stops in its last position
  /* enter your comments here:
  */
  
  // All we have to do is check to see if the mapped Distance is greater than 0
  // if so then we record it to the servo motor
  if (mapDistVal > 0)
    myservo.write(mapDistVal);


  // ---------- light sensor controls
  // read light sensor
  lightVal = analogRead(lightPin);  // light sensor
  roomLightReadings[sampleIndex] = lightVal;
  //baseLightVal = calcAvgValues(sampleIndex, roomLightReadings);

  // The following two lines designate a deadzone band for the blue led between .2 and .4 of the baseLightVal for no change.
  //if      ((lightVal > (baseLightVal - (0.3 * baseLightVal)))) { digitalWrite(lightLed, HIGH); }  // if val above thresH (20% below room light)
  //else if ((lightVal < (baseLightVal - (0.5 * baseLightVal)))) { digitalWrite(lightLed, LOW);  }  // if val under thresh (40% below room light)
  // observe when the blue led turns off and on as you move your finger up and down above the light sensor

  // ********** requirement #2
  // change the code above to expand the range from 0.3-0.5 to the max and explore the changes. 
  /* enter your comments here:
  */

  // The following two lines designate a deadzone band for the blue led between 
  // Below to get the Max value we just remove the 0.3 and 0.5 expression which removes
  // the range checking and takes the values as is.
  if      ((lightVal > (baseLightVal ))) { digitalWrite(lightLed, HIGH); }  
  else if ((lightVal < (baseLightVal ))) { digitalWrite(lightLed, LOW);  }  
  

  // ---------- pwm controls for the yellow led, where led dims from 100% to 0% based on the light value
  // pwmVal =lightVal;                         // copy the light val
  // pwmVal = min(pwmVal, baseLightVal);  // allow pwmVal value to be max at room light
  // pwmVal = max(pwmVal, 100);            // allow pwmVal value to be min is 10
  // mapPwmVal = map(pwmVal, 100, baseLightVal, 10, 245);
  // analogWrite(lightPwmLed, mapPwmVal);

  // ********** requirement #3
  // modify the code above to reverse the effect of dimming, that is when you cover the light sensors the yellow led should be at brightes
  /* enter your comments here:
  */

  pwmVal = lightVal;                         // copy the light val
  pwmVal = min(pwmVal, 900);  // allow pwmVal value to be max at no light
  pwmVal = max(pwmVal, 100);            // allow pwmVal value to be min is 10
  mapPwmVal = map(pwmVal, 100, 900, 245, 10);  // reverse the mapping
  analogWrite(lightPwmLed, mapPwmVal);


  // ---------- touch sensor controls
  // read the touch sensor, and if touched, turn on the green led, when no touch turn led off
  touchVal = analogRead(touchPin);
  //if (touchVal < (900)          ) { digitalWrite(touchLed, HIGH); }              // if touched
  //else                            { digitalWrite(touchLed, LOW);  }

  // ********** requirement #4
  // the code above uses an absolute value to detect touching which may not be suitable for your skin
  // change that value to a percentage of the baseline value based on the actual reading when you touch
   /* enter your comments here:
  */

  // First thing I wanted to do was get collection of samples when I touch it:
  touchReadings[sampleIndex] = touchVal;

  // Next I wanted to see what the average is
  // using an average to first see what the ranges are...
  int avgTouchReadings = calcAvgValues(sampleIndex, touchReadings);

  // Next I took the highest and lowest averages and divided them by 1023 to get percentage values
  // the values were 0.13 and 0.44 respectively
  double lowTouchRange = baseTouchVal * 0.13;
  double highTouchRange = baseTouchVal * 0.44;

  // ********** requirement #5
  // change the code so that every time you touch, the led will toggle, and when you untouch, led stays the same. hit: use a flag
  /* enter your comments here:
  */
  // so check touch range, if within range it's considered a touch
  if ((touchVal >= lowTouchRange ) && (touchVal <= highTouchRange))
  {
    //set the flag
    toggleTouch = !toggleTouch;
  }

  if(toggleTouch)
  { digitalWrite(touchLed, HIGH); }  // if touched
  else
  { digitalWrite(touchLed, LOW);  }

  
  // ---------- temperature sensor controls
  // read the temo sensor, and if temp higher than the baseline, turn red led on, else turn led off
  // you can increase the sensor temo by touching/sqweezing it between your thinp and index fingure
  tempVal = analogRead(tempPin);  // tmp sensor
  //if (tempVal > 1.05 * baseTempVal) { digitalWrite(tempLed, HIGH);  } 
  //else                              { digitalWrite(tempLed, LOW);   }
  // if you have skin temperature that is less than the room basline temp, the led will not turn on
  // in this case you need to change the code so that if the temperature is higher/lower than the baseline, the led will turn on

  // ********** requirement #6 
  // change the code and apply the deadzone band concept instead od threshold value, so that the led will stop flickering 
  /* enter your comments here:
  */

  // Just choose a 4% - 8% range to determine when to turn the Red light on
  // so it doesn't flip back and forth on specific temerature
  if (tempVal > 1.08 * baseTempVal) { digitalWrite(tempLed, HIGH);  } 
  if (tempVal < 1.04 * baseTempVal) { digitalWrite(tempLed, LOW);   }
  

  // ---------- printing on serial monitor
 
  Serial.print("Distance Base:\t"); Serial.print(baseDistVal);   Serial.print("\tDistance Map :\t"); Serial.println(mapDistVal);

  Serial.print("Light Base:  \t"); Serial.print(baseLightVal); Serial.print("\tMapped Value:\t");
  Serial.print(mapPwmVal);  Serial.print("\tLight Read:  \t"); Serial.println(lightVal); 

  Serial.print("Touch base:  \t"); Serial.print(baseTouchVal); Serial.print("\tAvg value:   \t");
  Serial.print(avgTouchReadings);  Serial.print("\tTouch Read:  \t"); Serial.println(touchVal);

  Serial.print("Temp base:   \t"); Serial.print(baseTempVal);   Serial.print("\tTemp Read:\t"); Serial.println(tempVal);
  

  // ---------- the mian loop dealy
  // here we are usning a loop delay to slow down the update rate of the analog input
  //delay(100);  // wait for 100ms, the invinit loop interval 

  // ********** requirement #7
  // change the delay value (10, 100, 500 1000), and state you observation on the various loop delays
  /* enter your comments here:
  */
  // When choosing 10, the touch sensor doesn't work well it loops too fast. Lights blink too fast.
  // when choosing 500, you have to hold down too long for the touch sensor to work, and the light
  // sensor takes a long time to be affected.
  // 250 seemed just like a good enough time for all sensors to work well.
  delay(250);


  // used for caculating averages inside an array
  // use the below to increase the index counter and 
  // bounds checking
  sampleIndex++;
  if (sampleIndex >= BUFFER_SIZE  )
    sampleIndex = 0;
}
