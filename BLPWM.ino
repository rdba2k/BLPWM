/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 BLPWM is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/

// Convert the PWM signal (15k-40kHz) from the brushed motor output of the FC between the duty cycle of 0% and 100%
// to a standard 50Hz brushless motor ESC signal between the pulse width of 960us to 2000us.
// the lower pulse width of 960us is to ensure the complete stop of the motor.

//#define DEBUG

#include "RP2040_PWM.h"

const int PWM_INPUT_PIN = 15; // input pin
const int PWM_OUTPUT_PIN = 7; // output pin
const int FREQ_OUTPUT = 50;   // output frequency
const int NO_PULSE_DURATION_THRESHOLD = 1000000;   // when the duty cycle is either 0% or 100%

const float minOutputDutyCycle = 4.8;   // %, 50Hz, 960us pulse width
const float maxOutputDutyCycle = 10.;  // %, 50Hz, 2000us pulse width
const float rangeOutputDutyCycle = 5.2; // %, 10 - 4.8
const short PWM_SAMPLES = 20;

volatile uint64_t highStartTime = 0;
volatile uint64_t lowStartTime = 0;
volatile uint64_t highDuration = 0;
volatile uint64_t lowDuration = 0;
volatile int pulseLevel = 0;
volatile float dutyCycle = 0;

RP2040_PWM* PWM_Output;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  pinMode(PWM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), handlePWMInput, CHANGE);

  lowStartTime = rp2040.getCycleCount64();  // initial signal low
  pulseLevel = 0;
}

void loop() {
}

// use the second core to output,
// the first core may possibly freeze for a moment when the interrupt runs.
void setup1() {
  delay(10);

  PWM_Output = new RP2040_PWM(PWM_OUTPUT_PIN, FREQ_OUTPUT, minOutputDutyCycle);
  if (PWM_Output)
  {
    PWM_Output->setPWM();
  }
}

void loop1() {
  short count = 0;
  float dutyCycleSum = 0.;

  while ( count < PWM_SAMPLES ) {
    // Periodically print the duty cycle, or use it for control
    float currentDutyCycle = dutyCycle; // Get a stable copy
#ifdef DEBUG
    Serial.print("Duty Cycle: ");
    Serial.print(currentDutyCycle);
    Serial.print("%");
    Serial.print("  highDuration: ");
    Serial.print(highDuration);
    Serial.print("  lowDuration: ");
    Serial.println(lowDuration);
#endif

    if ( ( digitalRead(PWM_INPUT_PIN) == HIGH ) && ( rp2040.getCycleCount64() - highStartTime > NO_PULSE_DURATION_THRESHOLD ) ) {
      currentDutyCycle = 100.;
#ifdef DEBUG
      Serial.print("Overwritten Duty Cycle: ");
      Serial.print(currentDutyCycle);
      Serial.println("%");
#endif
    } else if ( ( digitalRead(PWM_INPUT_PIN) == LOW ) && ( rp2040.getCycleCount64() - lowStartTime > NO_PULSE_DURATION_THRESHOLD ) ) {
      currentDutyCycle = 0.;
#ifdef DEBUG
      Serial.print("Overwritten Duty Cycle: ");
      Serial.print(currentDutyCycle);
      Serial.println("%");
#endif
    }
    dutyCycleSum += currentDutyCycle;
    count++ ;
  }

  float dutyCycleAvg = dutyCycleSum / PWM_SAMPLES;
#ifdef DEBUG
  Serial.print("Averaged Duty Cycle: ");
  Serial.print(dutyCycleAvg);
  Serial.println("%");
#endif

  float outputDutyCycle =  dutyCycleAvg * rangeOutputDutyCycle / 100. + minOutputDutyCycle;
  PWM_Output->setPWM(PWM_OUTPUT_PIN, FREQ_OUTPUT, outputDutyCycle);
}

void handlePWMInput() {
  if ( pulseLevel == 0 && digitalRead(PWM_INPUT_PIN) == HIGH) {
    highStartTime = rp2040.getCycleCount64();
    pulseLevel = 1;
    // Calculate low duration from previous cycle
    lowDuration = highStartTime - lowStartTime;
  } else if ( pulseLevel == 1 && digitalRead(PWM_INPUT_PIN) == LOW) { // LOW
    lowStartTime = rp2040.getCycleCount64();
    pulseLevel = 0;
    if (highStartTime > 0) { // Calculate high duration
      highDuration = lowStartTime - highStartTime;
      uint64_t totalDuration = highDuration + lowDuration;
      dutyCycle = (float)highDuration / totalDuration * 100.0;
    }
  }
}
