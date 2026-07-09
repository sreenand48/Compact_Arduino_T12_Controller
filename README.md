# Compact_Arduino_T12_Controller
A low-cost DIY Hakko T12 tip heater assembled using commonly available parts.
Only specialized components are the T12 Tip, T12 Handle and GX12 socket.


## Schematics & PCB Layout
Here is the schematic of the control circuit. The component list can be downloaded from [here](https://github.com/sreenand48/Compact_Arduino_T12_Controller/blob/main/T12_BOM.csv)
<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Schematic.png" alt="Front" width="720"> 

Here is my compact layout that I made on a perf board

<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/Image1.jpeg" alt="Front" width="180"> 
<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/Image2.jpeg" alt="Front" width="180"> 
<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/Image3.jpeg" alt="Front" width="180"> 
<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/Image4.jpeg" alt="Front" width="180"> 


## Notes

- The circuit does not explicitly require an I2C LCD but it preferrable for the sake of calibration/troubleshooting and general usage
- The current PID values are extremely conservative. Additional tuning may be required based on tip unsigned
- The main MOSFET does not require a heatsink, but the LM7805 regulator will require a heatsink especially is an LCD with backlight is unsigned
- Simple DuPont connectors can be used to connect the arduino and LCD, but the main power rails need extra solder for current carrying capacity upto 3 Amps
- The female GX12 connector should have atleast 3 pins and match up to the T12 handle socket

## Power Chart

Here is the power supply specifications needed to power a T12 tip. It is recommeded to have a headroom of 0.5 Amps or more to account for 5V linear regulator. 

<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/Power_Ratings.webp" alt="Front" width="500"> 

## T12 Pinout

Here is the T12 tip Pinout. 

| Pin | Schematic Net|
|----------|--------|
| `P+` | T12_Pinout |
| `P-` | GND|
| `Earth` | 470K to GND |

<img src="https://raw.githubusercontent.com/sreenand48/Compact_Arduino_T12_Controller/refs/heads/main/Images/T12_Pinout.png" alt="Front" width="500"> 




## Code

```c
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/sleep.h>

//IMPORTANT : This value has to be calibrated by plotting the
// the relative temprature diffrence from room temp and tip temprature.
// when compared to the ADC reading.
// Current gain is 220x as the T12 thermocouple produces 22uV/ °C
// and 1 count for the arduino ADC is 5V/1024=4.8mV.
// So the magnification is 4.8mV/22uV ≈ 220 based on experimentation
// The value may be 0.85<adcPerDegreeC<1.15

const float adcPerDegreeC = 1.05;

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int TEMP_SENSOR_PIN = A0;
const int SETPOINT_PIN = A1;
const int HEATER_PWM_PIN = 10;


const float TEMP_MIN_C = 200;
const float TEMP_MAX_C = 450;


const float KP = 11.0;
const float KI = 3.0;
const float KD = 1.0;

const int PWM_MAX = 255;
const int PWM_MIN = 0;


const unsigned long PID_SAMPLE_TIME_MS = 100;
const int ADC_NUM_SAMPLES = 32;
const int SETTLE_TIME = 950;
const float SMOOTH = 0.7;


float setpointTemp = 0;
float currentTemp = 0;
float oldTemp = 0;
float pidOutput = 0;

float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

unsigned long lastPidTime = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_MS = 150;

int readAveragedADC(int pin) {
  long sum = 0;
  for (int i = 0; i < 32; i++) {
    sum += analogRead(pin);
  }
  return sum >> 5;
}

float convertADCtoTemperature(int adcValue) {

  return adcValue / adcPerDegreeC;
}

float computePID(float setpoint, float actual, float dt) {

  error = setpoint - actual;
  integral += error * dt;
  float integralMax = PWM_MAX / (KI > 0 ? KI : 1);
  if (integral > integralMax) integral = integralMax;
  if (integral < -integralMax) integral = -integralMax;

  float tempDerivative = -(actual - oldTemp) / dt;
  derivative = 0.9 * derivative + 0.1 * tempDerivative;
  float output = (KP * error) + (KI * integral) + (KD * derivative);
  output = constrain(output, PWM_MIN, PWM_MAX);
  lastError = error;
  return output;
}

void applyHeaterOutput(float pidOutput) {
  int pwmCommand = 0;

  if (setpointTemp < 50) {
    pwmCommand = 255; 
  } else {
    pwmCommand = 255 - (int)pidOutput;
  }

  analogWrite(HEATER_PWM_PIN, pwmCommand);
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print((int)setpointTemp);
  lcd.print("C A:");
  lcd.print((int)currentTemp);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print((int)pidOutput);
}

void setup() {
  pinMode(TEMP_SENSOR_PIN, INPUT);
  pinMode(SETPOINT_PIN, INPUT);
  pinMode(HEATER_PWM_PIN, OUTPUT);
  digitalWrite(HEATER_PWM_PIN, HIGH);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("T12 PID Station");
  lcd.setCursor(0, 1);
  lcd.print("Safe Mode: OFF");
  delay(200);
  lcd.clear();

  lastPidTime = millis();
  lastDisplayUpdate = millis(); 
}

void loop() {
  unsigned long currentTime = millis();

  int setpointADC = analogRead(SETPOINT_PIN);
  
  setpointTemp = map(setpointADC, 0, 1023, TEMP_MIN_C, TEMP_MAX_C);

  if (currentTime - lastPidTime >= PID_SAMPLE_TIME_MS) {
    float dt = (currentTime - lastPidTime)/1000.0;
    lastPidTime = currentTime;

    digitalWrite(HEATER_PWM_PIN, HIGH);
    
    delayMicroseconds(950);
    int tempADC = readAveragedADC(A0);
    float rawTemp = convertADCtoTemperature(tempADC);
    oldTemp = currentTemp;
    currentTemp = (SMOOTH * rawTemp) + ((1 - SMOOTH) * oldTemp);

    pidOutput = computePID(setpointTemp, currentTemp, dt);

    applyHeaterOutput(pidOutput);


  }

  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_MS) {
    lastDisplayUpdate = currentTime;
    updateDisplay();
  }

}

```


## Additional Thanks
I would like the thank the following creators who helped and inspired the design of the T12 Controller
- [Stefan Wagner](https://github.com/wagiminator/ATmega-Soldering-Station)
- [Aka Kasyan TV](https://www.youtube.com/watch?v=UL6iDRJcY4I)
- [Electronoobs](https://electronoobs.com/tutorial/portable-arduino-soldering-iron-v3)
- [HomeMade Projects](https://www.youtube.com/watch?v=6qfoCy1P5nM)
- [Marco Reps](https://www.youtube.com/watch?v=GYIiOkr6x9o)
- [Great Scott!](https://www.youtube.com/watch?v=UvH49nzpJts)


## Additional Improvements

- Use a few buttons to add additional features like sleep, internal calibration, and emergency stop
- Use a boost converter to power the MOSFET gate instead of the bootstrap capacitor for better true 100 percent power delivery
- Use a mini buck converter before the linear regulator for more efficiency and removing heatsink
- 3D Print a case and utilize standard barrel connectors for circuit board Vcc
