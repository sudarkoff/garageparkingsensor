/**
 * Garage Parking Sensor
 */

#include <avr/sleep.h>
#include <avr/wdt.h>

#include <Bounce.h>
#include <EEPROM.h>

// Arduino convenience functions.
// https://github.com/sudarkoff/hardwin
#define USE_EEPROM
#include <hardwin.h>

// Uncomment to enable debug output to serial
//#define DEBUG

#define NUM_READS 1                // PING))) is accurate enough, no need for averaging
#define USEC_PER_CM 29             // microseconds per centimeter
#define EPSILON (USEC_PER_CM * 10) // If motion is less than 10 cm, we ignore it
#define SLEEP_WAITING_PERIOD 3000  // Period of inactivity before ATmega goes to sleep
#define SAVE_WAITING_PERIOD 5000   // To prevent frequent writes to EEPROM (like saving distance)

const int pingPin = 9;             // The pin PING))) sensor is connected to
const int savePin = 10;            // The pin Save button is connected to

const int distanceLed[] = {2, 3, 4, 5, 6, 7};
                                   // Pins for LEDs of proximity indicator
const int distanceLimitAddr = 0;   // EEPROM address of the distance threshold parameter
long distanceLimit;                // Local instance of the distance threshold parameter (cm)
long distanceMax = 300;            // 300 cm is the maximum distance PING))) can detect

// De-bouncer for Save button
Bounce saveButton = Bounce(savePin, 5);

// Global flag for ATmega Watchdog
volatile boolean f_wdt = 1;

/**
 * Arduino setup routine.
 */

void
setup()
{
  // Initialize proximity indicator pins
  for (int i=0; i<6; ++i) {
    pinMode(distanceLed[i], OUTPUT);
  }

  // Initialize Save button pin
  pinMode(savePin, INPUT);
  // Read the distance threshold value from EEPROM (default is 20 cm)
  hardwin::EEPROM_read(distanceLimitAddr, distanceLimit,
    static_cast<long>(3), static_cast<long>(300), static_cast<long>(20));

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)
  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode

  // Watchdog timer: 2 seconds appears to be responsive enough
  setup_watchdog(7);
  
#ifdef DEBUG
  Serial.begin(9600);
#endif
}

/**
 * Main loop
 */

void
loop()
{
  static long prevDuration = 0;
  static long prevDurationAt = 0;
  static long lastSaved = 0;
  static boolean sleep_mode = false;

  // Read the sensor NUM_READS times and take the average discarding outliers
  long duration = readDuration(NUM_READS);

  // Convert duration to distance
  long cm = usec2cm(duration);

  // If the distance changed by more than EPSILON ...
  if (!hardwin::equal(duration, prevDuration, EPSILON)) {
    // Note the last change
    sleep_mode = false;
    prevDuration = duration;
    prevDurationAt = millis();
  } else {
    // Otherwise, if there was no change for at least SLEEP_WAITING_PERIOD microseconds
    if (abs(millis() - prevDurationAt) > SLEEP_WAITING_PERIOD) {
      // go to sleep
      sleep_mode = true;
      sleep();
    }
  }

  // If there was change in distance (car is moving)
  if (!sleep_mode) {
    // Update the proximity indicator
    updateProximity(cm);
  }

  // If the Save button is pushed
  saveButton.update();
  if (saveButton.read() == HIGH) {
    // And we didn't save the distance threshold in the last SAVE_WAITING_PERIOD microseconds
    if (millis() - lastSaved > SAVE_WAITING_PERIOD) {
      // save the new distance threshold
      hardwin::EEPROM_write(distanceLimitAddr, cm);
      distanceLimit = cm;
      lastSaved = millis();

      // and flash the RED LED 3 times as a confirmation
      int states[] = {LOW, LOW, LOW, LOW, LOW, LOW};        // OFF
      switchLeds(states);
      hardwin::flashLed(distanceLed[5], 3, 50);
    }
  }

  delay(75);
}

/**
 * Read PING))) sensor
 *
 * Result:
 *   time it took the sound to travel to the target and back in microseconts
 */

long
readPing()
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  return duration;
}


/**
 * Read PING))) sensor a number of times (up to 10) and return the average while discarding outliers.
 *
 * Note:
 *   I wrote this function trying to combat the "accuracy issue". It turns out the issue was my power
 *   supply that couldn't source enough current to power PING))), so it was producing wrong results.
 *   Replacing the power supply fixed the issue and there's no need to take more than one measurement.
 */

long
readDuration(long times)
{
  long sum = 0;
  static long durationArray[10];

  for (int i = 0; i < times; i++) {
    durationArray[i] = readPing();
#ifdef DEBUG
    Serial.print(durationArray[i]); Serial.print(", ");
#endif
    sum += durationArray[i];
  }
  long mean = sum / times;
#ifdef DEBUG
  Serial.println(mean);
#endif

#if 0
  // Discard outliers using a very simple heuristic:
  //   if a value differs from the mean by more than EPSILON, discard it
  long good_times = times;

  for (int i = 0; i < times; i++) {
    if (abs(durationArray[i] - mean) > EPSILON) {
#ifdef DEBUG
      Serial.print("outlier: "); Serial.println(durationArray[i]);
#endif
      sum -= durationArray[i];
      good_times--;
    } 
  }
  
  mean = sum / good_times;
#endif // 0

  return mean;
}


/**
 * Convert time delay in microseconds it took the sound to travel to the target and back to centimeters.
 */

long
usec2cm(long usec)
{
  // Speed of sound is 340 m/s or 29 usec/cm.
  // The sound travels out and back, so devide the distance by 2.
  return usec / 29 / 2;
}


/**
 * Update the proximity indicator LEDs.
 */
 
void
updateProximity(long distance)
{
  double rangeThreshold[] = {
    distanceMax,
    distanceLimit + (distanceMax - distanceLimit) * 0.65,
    distanceLimit + (distanceMax - distanceLimit) * 0.40,
    distanceLimit + (distanceMax - distanceLimit) * 0.20,
    distanceLimit + (distanceMax - distanceLimit) * 0.05,
    distanceLimit
  };

  if (distance > rangeThreshold[0]) {
    int states[] = {LOW, LOW, LOW, LOW, LOW, LOW};        // OFF
    switchLeds(states);
  }
  else if (distance > rangeThreshold[1]) {
    int states[] = {HIGH, LOW, LOW, LOW, LOW, LOW};       // GREEN1
    switchLeds(states);
  }
  else if (distance > rangeThreshold[2]) {
    int states[] = {HIGH, HIGH, LOW, LOW, LOW, LOW};      // GREEN2
    switchLeds(states);
  }
  else if (distance > rangeThreshold[3]) {
    int states[] = {HIGH, HIGH, HIGH, LOW, LOW, LOW};     // GREEN3
    switchLeds(states);
  }
  else if (distance > rangeThreshold[4]) {
    int states[] = {HIGH, HIGH, HIGH, HIGH, LOW, LOW};    // YELLOW1
    switchLeds(states);
  }
  else if (distance > rangeThreshold[5]) {
    int states[] = {HIGH, HIGH, HIGH, HIGH, HIGH, LOW};   // YELLOW2
    switchLeds(states);
  }
  else {
    int states[] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};  // RED
    switchLeds(states);
  }
}


/**
 * Turn on/off the specified LEDs
 */

void
switchLeds(int state[])
{
  for (int i = 0; i < 5; ++i) {
    digitalWrite(distanceLed[i], state[i]);
  }
  if (state[5] == HIGH) {
    hardwin::flashLed(distanceLed[5], 1, 50);
  } else {
    digitalWrite(distanceLed[5], LOW);
    delay(50);
  }
}


/**
 * Put ATmega to sleep (see setup() for details).
 */

void
sleep()
{
  // Turn off the proximity indicator
  int states[] = {LOW, LOW, LOW, LOW, LOW, LOW};
  switchLeds(states);

  // Switch ADC OFF
  cbi(ADCSRA,ADEN);

  // Put ATmega to sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // zzzzz.....

  // Wake up
  sleep_disable();

  // Switch ADC ON
  sbi(ADCSRA,ADEN);
}


/**
 * Setup the watchdog to wake up ATmega after a specified period of time:
 *
 * 0 = 16 ms
 * 1 = 32 ms
 * 2 = 64 ms
 * 3 = 128 ms
 * 4 = 250 ms
 * 5 = 500 ms
 * 6 = 1 sec
 * 7 = 2 sec
 * 8 = 4 sec
 * 9 = 8 sec
 */

void
setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
 
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}


/**
 * Watchdog Interrupt Service is executed when watchdog timed out
 */

ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}

