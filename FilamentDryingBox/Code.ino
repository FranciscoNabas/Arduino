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
* A digital rotary encoder to control the target temperature.
* A 20/4 LCD display with I2C module.
*
* In my case with all this aparatus the peak current draw was approximatelly 4.4Amps.
*
* Connections:
* DTH22 data pin       : 12.
* Heating element pin  : 11.
* Heatsink fan pin     : 10.
* DS18B20 data pin     : 9.
* Encoder CLK          : 5.
* Encoder DT           : 5.
* Encoder SW           : 2.
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

// #include <Arduino_FreeRTOS.h>
// #include <semphr.h>

#include <DHT.h>
#include <Wire.h>
#include <DHT_U.h>
#include <KY040.h>
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// Defining temperature thresholds.
#define MIN_TEMP         ((double)30.00)  // Minimum target temperature.
#define MAX_TEMP         ((double)70.00)  // Maximum target temperature.
#define TEMP_TGT_DIFF    05.00            // Temperature differential between the heatsink and ambient temperature. Used to decide if we shut off the fan.
#define HS_MAX_TEMP      80.00            // Heatsink maximum temperature.
#define HS_TEM_DIFF_CUT  10.00            // Heatsink temperature cutoff. Used to decide if we shut off the fan.

// Defining DHT22 pins and type.
#define DHTPIN           12
#define DHTTYPE          DHT22
#define EULER            2.718281828459045235360287471352

// IO pins.
#define PIN_TSENS        9     // The heatsink temperature sensor data pin.
#define PIN_POT          A7    // The analog pin where the potentiometer is connected. To control the target temperature (pre-encoder, not used anymore).
#define PIN_FAN          10    // The heatsink fan pin.
#define PIN_H_ELEMENT    11    // The heating element pin.
#define PIN_ENC_DT       6     // The target temperature encoder pin 'B'.
#define PIN_ENC_CLK      5     // The target temperature encoder pin 'A'.
#define PIN_ENC_SW       2     // The target temperature encoder switch pin.

// Fan pin PWM value.
#define HS_FAN_SPEED     255

// The KY040 rotary encoder uses internal switches to signal changes.
// Like all mechanical switches these bounce, I.E. you turn the encoder once or press it once and
// the encoder sends more than one signal.
// The 'KY040.h' library does a terrific job avoiding bounces when turning the pot, but it doesn't support
// the switch. So we implement a very rudimentary debouncing algorithm rejecting any extra signal that comes
// before the debounce timeout.
#define DEBOUNCE_MS      250

// An enum representing the heatsing fan mode.
enum class heatsink_fan_mode_t {
  ON,       // The fan is always on.
  OFF,      // The fan is always off.
  AUTO,     // The fan is on or off according to our algorithm.
};

// Global variables.
bool startup             = true;                      // Used to determine if the system was booted for the first time.
bool fan_on              = false;                     // True if the heatsink fan is on.
float temperature        = 0.0;                       // The ambient temperature.
float relative_humidity  = 0.0;                       // The ambient relative humidity.
float absolute_humidity  = 0.0;                       // The ambient absolute humidity.
float heatsink_temp      = 0.0;                       // The heatsink temperature.
int enc_pina_prev        = 0;                         // Variable to store the previous state of the encoder 'A' (CLK) pin.
double input, set_point, output;                      // Variables used by the PID controller. input == temperature, set_point == target, output == PID output.

volatile double target_temp;                          // The raw target temperature set by the encoder's position.
heatsink_fan_mode_t hs_prev_mode;                     // The previous fan mode.
volatile heatsink_fan_mode_t hs_fan_mode;             // The heatsink fan mode to be set by the ISR.
volatile unsigned long debounce_millis = millis();    // Stores the milliseconds since the controler was turned on so we can apply to the debouncer.

// Creating devices.
DeviceAddress tsens_addr;                               // Used by the heatsink temperature sensor.
OneWire one_wire(PIN_TSENS);                            // Used by the heatsink temperature sensor.
DallasTemperature tsens(&one_wire);                     // Used by the heatsink temperature sensor.
DHT_Unified dht(DHTPIN, DHTTYPE);                       // Used by the DHT22 sensor.
LiquidCrystal_I2C lcd(0x27, 20, 4);                     // Used to control the I2C LCD display interface.
PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);  // The PID controller.
KY040 encoder(PIN_ENC_CLK, PIN_ENC_DT);                 // The target temperature control encoder.

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

/**
* @brief Enables interrupts for the pin.
*
* @param pin The pin GPIO number to enable interrupts.
* @returns Nothing.
*/
void pci_setup(byte pin) {
  // Setting the bit corresponding to the pin on.
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));

  // Clear any outstanding interrupts.
  PCIFR |= bit(digitalPinToPCICRbit(pin));

  // Enable interrupts for the group.
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

/**
* @brief The ISR (Interrupt Service Routine) to be called when the interrupt-enabled pins state changes (PCINT2_vect).
*/
ISR(PCINT2_vect) {
  static bool first = true;

  // Getting the encoder state to check if it was rotated.
  switch (encoder.getRotation()) {
    case KY040::CLOCKWISE:
      target_temp++;
      break;

    case KY040::COUNTERCLOCKWISE:
      target_temp--;
      break;
  }

  // Checking if the encoder switch was pressed.
  if (digitalRead(PIN_ENC_SW) == LOW) {
    unsigned long current_millis = millis();

    // If the debouncing period elapsed or this is the first time we press the button we change the fan mode.
    if (current_millis - debounce_millis >= DEBOUNCE_MS || first) {
      switch (hs_fan_mode) {
        case heatsink_fan_mode_t::AUTO:
          hs_fan_mode = heatsink_fan_mode_t::ON;
          break;

        case heatsink_fan_mode_t::ON:
          hs_fan_mode = heatsink_fan_mode_t::OFF;
          break;

        case heatsink_fan_mode_t::OFF:
          hs_fan_mode = heatsink_fan_mode_t::AUTO;
          break;
      }

      // Saving the millis since we changed the mode.
      debounce_millis = current_millis;
      if (first)
        first = false;
    }
  }
}

void setup() {
  // Initializing serial communication.
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println("####################\n##    Starting    ##\n####################");
  Serial.println();
  Serial.println();
  Serial.println();

  // Setting GPIO pin modes.
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_H_ELEMENT, OUTPUT);

  // Enabling interrupts for the encoder's pins.
  pci_setup(PIN_ENC_CLK);
  pci_setup(PIN_ENC_DT);
  pci_setup(PIN_ENC_SW);

  // Setting the heatsink fan mode to automatic.
  hs_fan_mode = hs_prev_mode = heatsink_fan_mode_t::AUTO;

  // Turning OFF the heatsink fan.
  analogWrite(PIN_FAN, 0);

  // Setting the setpoint to the minimum target temperature and configuring the PID.
  set_point = target_temp = ((MAX_TEMP - MIN_TEMP) / 2) + MIN_TEMP;
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
  static heatsink_fan_mode_t our_fan_mode;

  // Reading the volatile variables set by the interrupt.
  // During the reading we disable interrupts (cli()) to avoid race conditions.
  cli();

  // Setting the PID set point based on the raw target temperature. We do this so we don't pass our minimum and maximum.
  set_point = target_temp >= MIN_TEMP ? target_temp <= MAX_TEMP ? target_temp : MAX_TEMP : MIN_TEMP;

  // Resetting the raw target temperature.
  target_temp = set_point;

  // Saving the fan mode so we can analyze it.
  our_fan_mode = hs_fan_mode;
  
  // Re-enabling interrupts.
  sei();

  if (our_fan_mode != hs_prev_mode) {
    // Fan mode changed, so we print the status on the display.
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("Heatsink fan mode:");

    switch (our_fan_mode) {
      case heatsink_fan_mode_t::AUTO:
        lcd.setCursor(8, 2);
        lcd.print("AUTO");
        break;

      case heatsink_fan_mode_t::ON:
        lcd.setCursor(9, 2);
        lcd.print("ON");
        break;

      case heatsink_fan_mode_t::OFF:
        lcd.setCursor(8, 2);
        lcd.print("OFF");
        break;
    }

    delay(1000);
    lcd.clear();

    // Saving the current mode.
    hs_prev_mode = our_fan_mode;
  }

  // We print all the status twice in the loop. We do that because when all the sensors are functioning it takes
  // some time to perform all the readings, calculations, and controlling the temperature.
  // If the user rotates the encoder it takes some time for the value to reflect in the LCD display.
  // Printing the status here makes it more responsive, but it prints the old values. That's why we print a second time later.
  print_status();

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
  } else {
    temperature = 0.0;
    relative_humidity = 0.0;
  }

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
  //
  // The algorithm is overriden by the value of 'hs_fan_mode'.
  switch (our_fan_mode) {
    // Mode ON. The fan is always on.
    case heatsink_fan_mode_t::ON:
      {
        if (!fan_on) {
          analogWrite(PIN_FAN, HS_FAN_SPEED);
          fan_on = true;
        }
      }
      break;

    // Mode OFF. The fan is always off.
    case heatsink_fan_mode_t::OFF:
      {
        if (fan_on) {
          analogWrite(PIN_FAN, 0);
          fan_on = false;
        }
      }
      break;

    // Mode AUTO. The fan is on or off depending on the environmental conditions.
    case heatsink_fan_mode_t::AUTO:
      {
        if (startup) {
          // We are starting up, so we only turn on the fan if the heatsink temperature is greater than our maximum
          // or the ambient temperature is greater than the target minus the differential.
          if (heatsink_temp >= HS_MAX_TEMP || temperature >= set_point - TEMP_TGT_DIFF) {
            analogWrite(PIN_FAN, HS_FAN_SPEED);
            startup = false;
            fan_on = true;
          }
          else {
            // We are starting up, but our temperatures are not ideal. So we turn off the fan.
            if (fan_on) {
              analogWrite(PIN_FAN, 0);
              fan_on = false;
            }
          }
        }
        else {
          // We are not starting up, so if the fan is on we turn it off when the heatsink temperature is smaller than the target plus the cuttoff
          // and the ambient temperature is smaller than the target minus the differential.
          if (fan_on && heatsink_temp <= set_point + HS_TEM_DIFF_CUT && temperature <= set_point - TEMP_TGT_DIFF) {
            analogWrite(PIN_FAN, 0);
            fan_on = false;
          }

          // The fan is off, so we turn it on if the heatsink temperature is greater than our maximum or the ambient temperature is
          // greater than the target minus the differential.
          else if (!fan_on && (heatsink_temp >= HS_MAX_TEMP || temperature >= set_point - TEMP_TGT_DIFF)) {
            analogWrite(PIN_FAN, HS_FAN_SPEED);
            fan_on = true;
          }
        }
      }
      break;
  }

  // Printing the status again.
  print_status();

  // Delaying. This period is sometimes not enough for the heatsink sensor to update which causes
  // us not being able to read the temperature. But it's only once in a while.
  // To be honest there's no harm in increasing the delay here, other than the user's input responsiveness.
  delay(20);
}

/**
* @brief Prints the status to the LCD display. The status includes ambient, target, and heatsink temperatures, relative and absolute humidity,
*        heatsink fan status, and the PWM value being applied to the heating element.
*
* @returns Nothing.
*/
void print_status(void) {
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

  // Writing if the fan is on and the PWM value on the heating element pin.
  lcd.setCursor(0, 3);
  lcd.print("Fan on:");
  lcd.print(fan_on ? "Yes" : "No ");
  lcd.print(" PWM:");
  lcd.print((uint8_t)output);

  // Homing the LCD and delaying.
  lcd.home();
}

/**
* @brief Controls the heating element temperature with the PID controller.
*
* @returns Nothing.
*/
void control_temperature(void) {

  // Old target temperature set method, using an potentiometer and analog input.

  // Reading the value on the target temp potentiomenter and calculating the
  // percentage of which he is turned. The 'float' cast here is important before
  // doing the math.
  // int potValue = analogRead(PIN_POT);
  // float percent = (float)potValue / 1023;

  // Applying the percentage to the difference between the minimum and maximum target temperatures.
  // Then we assign to the setpoint before calling 'pid.Compute()'.
  // We cast to an 'int' here before casting to a 'double' so we keep the target temperature always in increments
  // of one degree. This is helpful because since we are using a simple potentiomenter and analog readings we are
  // succeptable to floating and interference. In a future version the potentiomenter will be replaced with a
  // digital rotary encoder to fancy up thinks a little bit and keep everything more stable.
  // set_point = (double)((int)(MIN_TEMP + ((MIN_TEMP - MAX_TEMP) * percent)));

  if (!isnan(temperature)) {
    // Computing the output PWM and applying to the heating element pin.
    input = temperature;
    pid.Compute();
    analogWrite(PIN_H_ELEMENT, output);
  }
}

/**
* @brief Gets the heatsink temperature in celsius by reading the DS18B20 sensor.
*
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
*
* @returns The absolute humidity.
*/
float get_abs_humidity(void) {
  // I have no idea what in the world this calculation does, got it from the interwebs =p.
  return ((6.112 * (pow(EULER, ((17.67 * temperature) / (temperature + 243.5)))) * relative_humidity * 2.1674) / (273.15 + temperature));
}