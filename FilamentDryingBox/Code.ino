/*
* Filament drying box controller.
*
* This code was based on this project:
* https://marlonnardi.com/2023/10/10/o-problema-que-acaba-com-suas-impressoes-3d-caixa-secadora-de-filamentos-dry-fila-box/
*
* The project consists of a heating element controlled by a PID and fans to keep
* the temperature inside a sealed environment to a target to help dry 3D printer plastic filaments.
* The code and PID controls the heating element temperature based on the ambient temperature.
*
* Parts used:
* Arduino Pro Mini 5V.
* DH22 Temperature and Humidity sensor.
* A hot end heating element with a heatsink.
* DS18B20 Temperature sensor for the heating element.
* 80mm / 80mm 12V fan for the heatsink.
* 12V / 10A power supply.
* A 100KB potentiomenter to control the target temperature.
* A 20/4 LCD display with I2C module.
*
* Connections:
* DTH22 data pin       : 12.
* Heating element pin  : 11.
* Heatsink fan pin     : 10.
* DS18B20 data pin     : 9.
* Potentiomenter       : A7.
*
* Circuit diagram can be found on the post where we based our project on.
*
* This code is distributed under the MIT license.
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
*/

#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// Defining temperature thresholds.
#define MIN_TEMP         30.00  // Minimum target temperature.
#define MAX_TEMP         70.00  // Maximum target temperature.
#define TEMP_TGT_DIFF    05.00  // Temperature differential between the heatsink and ambient temperature. Used to decide if we shut off the fan.
#define HS_MAX_TEMP      80.00  // Heatsink maximum temperature.
#define HS_TEM_DIFF_CUT  10.00  // Heatsink temperature cutoff. Used to decide if we shut off the fan.

// Defining DHT22 pins and type.
#define DHTPIN           12
#define DHTTYPE          DHT22
#define EULER            2.718281828459045235360287471352

// IO pins.
#define PIN_TSENS        9
#define PIN_POT          A7
#define PIN_FAN          10
#define PIN_H_ELEMENT    11

// Global variables.
bool startup             = true;    // Used to determine if the system was booted for the first time.
bool fan_on              = false;   // True if the heatsink fan is on.
int serial_count         = 0;       // Used to reduce the amount of times we write to the serial output for debugging purposes.
float temperature        = 0.0;     // The ambient temperature.
float relative_humidity  = 0.0;     // The ambient relative humidity.
float absolute_humidity  = 0.0;     // The ambient absolute humidity.
float heatsink_temp      = 0.0;     // The heatsink temperature.
double set_point, input, output;    // Variables used by the PID controller. input == temperature, set_point == target, output == PID output.

// Creating devices.
DeviceAddress tsens_addr;                               // Used by the heatsink temperature sensor.
OneWire one_wire(PIN_TSENS);                            // Used by the heatsink temperature sensor.
DallasTemperature tsens(&one_wire);                     // Used by the heatsink temperature sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);                       // Used by the DHT22 sensor.
LiquidCrystal_I2C lcd(0x27, 20, 4);                     // Used to control the I2C LCD display interface.
PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);  // The PID controller.

// Custom char 'degrees' for celsius temperature.
byte customChar[] = {
  0b00010,
  0b00101,
  0b00101,
  0b00010,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

void setup() {
  // Initializing serial communication.
  Serial.begin(9600);

  // Setting GPIO pin modes.
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_H_ELEMENT, OUTPUT);

  // Turning OFF the heatsink fan.
  analogWrite(PIN_FAN, 0);

  // Setting the setpoint to the minimum target temperature and configuring the PID.
  set_point = MIN_TEMP;
  pid.SetMode(AUTOMATIC);

  // Initializing sensors.
  dht.begin();
  tsens.begin();

  // Initializing LCD display.
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, customChar);
}

void loop() {
  // Reading ambient temperature.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temperature = event.temperature;

  // Reading ambient relative humidity.
  dht.humidity().getEvent(&event);
  relative_humidity = event.relative_humidity;

  // Reading heatsink temperature.
  heatsink_temp = get_heatsink_temperature();

  if (!isnan(temperature) && !isnan(relative_humidity)) {
    // We have temperature and rh. Let's get the absolute humidity and control the PID.
    absolute_humidity = get_abs_humidity();
    control_temperature();
  }
  else {
    temperature        = 0.0;
    relative_humidity  = 0.0;
  }

  // Printing ambient temperature.
  lcd.print("Tmp:");
  lcd.print(temperature);
  lcd.write(0);
  lcd.print("C/");
  lcd.print(set_point);
  lcd.write(0);
  lcd.print("C");

  // Printint relative and absolute humidity.
  lcd.setCursor(0, 1);
  lcd.print("RH:");
  lcd.print(relative_humidity);
  lcd.print("% AH:");
  lcd.print(absolute_humidity);
  lcd.print("%");

  // Printing heatsink temperature.
  lcd.setCursor(0, 2);
  lcd.print("HS temp:");
  lcd.print(heatsink_temp);
  lcd.write(0);
  lcd.print("C");

  // Algorithm to decide if we turn the heatsink fan on or off.
  // This is used because once on the heating element cannot keep up with the cooling when the ambient
  // temperature is too low. So we turn the fan on and off to allow optimal temperature transfer and circulation.
  //
  // Workflow:
  // - If we just booted up we wait the heatsink reach our maximum threshold temperature before turning the fan on.
  // - Then we check if the fan is on.
  //    - If it's on, the heatsink temperature is smaller than the target plus the differential, and the ambient temperature
  //      is lower than the target minus the difference it means the chamber and the heatsink is too cold, so we turn off the fan.
  //    - If the fan is off and the heatsink temperature reached our maximum we turn on the fan.
  //
  // This means we will allow the heating element to be nice and hot while the ambient temperature is low, and when the ambient
  // temperature is close enough to the target we keep the fan on to improve circulation.
  if (startup) {
    if (heatsink_temp >= HS_MAX_TEMP) {
      analogWrite(PIN_FAN, 255);
      startup  = false;
      fan_on   = true;
    }
  }
  else {
    if (fan_on && heatsink_temp <= set_point + HS_TEM_DIFF_CUT && temperature <= set_point - TEMP_TGT_DIFF) {
      analogWrite(PIN_FAN, 0);
      fan_on = false;
    }
    else if (!fan_on && heatsink_temp >= HS_MAX_TEMP) {
      analogWrite(PIN_FAN, 255);
      fan_on = true;
    }
  }

  // Writing if the fan is on and the PWM value on the heating element pin.
  lcd.setCursor(0, 3);
  lcd.print("Fan on:");
  if (fan_on) lcd.print("Yes");
  else lcd.print("No ");

  lcd.print(" PWM:");
  lcd.print((uint8_t)output);

  // Homing the LCD and delaying.
  lcd.home();

  delay(100);
}

/**
* @brief Controls the heating element temperature with the PID controller.
* @returns Nothing.
*/
void control_temperature(void) {
  // Reading the value on the target temp potentiomenter and calculating the
  // percentage of which he is turned. The 'float' cast here is important before
  // doing the math.
  int potValue = analogRead(PIN_POT);
  float percent = (float)potValue / 1023;

  // Applying the percentage to the difference between the minimum and maximum target temperatures.
  // Then we assign to the setpoint before calling 'pid.Compute()'.
  // We cast to an 'int' here before casting to a 'double' so we keep the target temperature always in increments
  // of one degree. This is helpful because since we are using a simple potentiomenter and analog readings we are
  // succeptable to floating and interference. In a future version the potentiomenter will be replaced with a
  // digital rotary encoder to fancy up thinks a little bit and keep everything more stable.
  set_point = (double)((int)(MIN_TEMP + ((MIN_TEMP - MAX_TEMP) * percent)));

  if (!isnan(temperature)) {
    // Computing the output PWM and applying to the heating element pin.
    input = temperature;
    pid.Compute();
    analogWrite(PIN_H_ELEMENT, output);
  }

  // Serial debugging.
  if (serial_count >= 5) {
    Serial.print("Pot: ");
    Serial.print(potValue);
    Serial.print("; Percent: ");
    Serial.print(percent * 100);
    Serial.print("%; Target: ");
    Serial.println(set_point);

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");

    Serial.print("PWM: ");
    Serial.println(output);
    Serial.println();

    serial_count = 0;
  }

  serial_count++;
}

/**
* @brief Gets the heatsink temperature in celsius by reading the DS18B20 sensor.
* @returns The temperature in degrees celsius.
*/
float get_heatsink_temperature(void) {
  // Asking the sensor to calculate the temperature.
  tsens.requestTemperatures();
  if (!tsens.getAddress(tsens_addr, 0))
    return 0.0;

  // Converting the temperature to celsius and returning.
  return tsens.getTempC(tsens_addr, 1);
}

/**
* @brief Converts relative humidity to absolute humidity.
* @returns The absolute humidity.
*/
float get_abs_humidity(void) {
  // I have no idea what in the world this calculation does, got it from the interwebs =p.
  return ((6.112 * (pow(EULER, ((17.67 * temperature) / (temperature + 243.5)))) * relative_humidity * 2.1674) / (273.15 + temperature));
}