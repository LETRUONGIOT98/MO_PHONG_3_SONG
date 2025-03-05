#include <TimerOne.h>           // Include TimerOne library for Timer1 operations
#include <Wire.h>              // Include Wire library for I2C communication
#include <LiquidCrystal_I2C.h> // Include LiquidCrystal_I2C library for LCD display

// Define button pins
const int nutFreqSquareTang = 2;  // Pin for increasing square wave frequency
const int nutFreqSquareGiam = 3;  // Pin for decreasing square wave frequency
const int nutDutySquareTang = 4;  // Pin for increasing square wave duty cycle
const int nutDutySquareGiam = 6;  // Pin for decreasing square wave duty cycle
const int tang_SX = 7;            // Pin for increasing frequency of the second square wave
const int giam_SX = 8;            // Pin for decreasing frequency of the second square wave
const int nutFreqSineTang = A1;   // Pin for increasing sine wave frequency
const int nutFreqSineGiam = A2;   // Pin for decreasing sine wave frequency
const int ok = 12;                // Pin for toggling the mode

// Define output pins
const int xung_CKP = 5;          // Pin for Timer0 output
const int squareWave2Pin = 9;    // Pin for Timer2 output
const int pwmOutputPin = 10;     // Pin for PWM output
const int delayPin = 11;         // Pin for delay signal

// Initialize LCD display
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Initialize LCD with I2C address 0x27 and 20x4 display

// Constants for sine wave algorithm
const float pi = 3.14159;          // Value of Pi
volatile float T = 100;            // Sample time in microseconds
volatile float sineFrequency = 100; // Sine wave frequency in Hz
const float A = 490;              // Amplitude of sine wave
float a[] = {0.0, A * sin(2 * pi * sineFrequency * T / 1000000.0), 0.0}; // Sine wave values
volatile float c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2)); // Coefficient for sine wave
int debounceDelay = 50;           // Debounce delay for button presses

// Parameters for the second square wave
volatile int square2Frequency = sineFrequency; // Frequency of the second square wave
volatile int square2DutyCycle = 10; // Duty cycle for the second square wave in percent

unsigned long previousMillis2 = 0;  // Previous time for second square wave
unsigned long interval2;            // Interval between pulses for second square wave
unsigned long onTime2;              // Time the LED is on for second square wave
unsigned long offTime2;             // Time the LED is off for second square wave
bool ledState2 = LOW;               // State of the LED for second square wave
unsigned long time1;                // Timing variables
unsigned long time2;
float soxung = 20.0;               // Number of pulses
float xungxuong = 5.0;             // Pulse width
float xungnghi = 2.0;              // Duty cycle of the pulse
int j = 0, L = 0;                  // State variables for timing
float i = 0.0, h = 0.0;            // Additional state variables
int cong = 0;                      // Mode state
float k = 0;                       // Additional counter
bool chedo = false;                // Mode toggle
volatile unsigned long isrTime;   // ISR time

void setup() {
  // Initialize input pins
  square2Frequency = sineFrequency; // Set initial frequency for the second square wave
  pinMode(nutFreqSquareTang, INPUT_PULLUP); // Set pin for increasing square wave frequency as input with pull-up resistor
  pinMode(nutFreqSquareGiam, INPUT_PULLUP); // Set pin for decreasing square wave frequency as input with pull-up resistor
  pinMode(nutDutySquareTang, INPUT_PULLUP); // Set pin for increasing square wave duty cycle as input with pull-up resistor
  pinMode(nutDutySquareGiam, INPUT_PULLUP); // Set pin for decreasing square wave duty cycle as input with pull-up resistor
  pinMode(tang_SX, INPUT_PULLUP);           // Set pin for increasing second square wave frequency as input with pull-up resistor
  pinMode(giam_SX, INPUT_PULLUP);           // Set pin for decreasing second square wave frequency as input with pull-up resistor
  pinMode(nutFreqSineTang, INPUT_PULLUP);   // Set pin for increasing sine wave frequency as input with pull-up resistor
  pinMode(nutFreqSineGiam, INPUT_PULLUP);   // Set pin for decreasing sine wave frequency as input with pull-up resistor
  pinMode(ok, INPUT_PULLUP);                // Set pin for toggling mode as input with pull-up resistor
  
  // Initialize output pins
  pinMode(xung_CKP, OUTPUT);          // Set pin for Timer0 output as output
  pinMode(squareWave2Pin, OUTPUT);    // Set pin for Timer2 output as output
  pinMode(pwmOutputPin, OUTPUT);      // Set pin for PWM output as output
  pinMode(delayPin, OUTPUT);          // Set pin for delay signal as output
  
  // Initialize LCD display
  lcd.init();                         // Initialize LCD
  lcd.backlight();                    // Turn on LCD backlight
  lcd.clear();                        // Clear LCD display
  Serial.begin(9600);                // Initialize serial communication at 9600 baud
  
  // Initialize Timer1 for sine wave
  Timer1.initialize(T);              // Set sample time for sine wave signal
  Timer1.pwm(pwmOutputPin, 0, T);   // Generate PWM signal on pin with period T and duty cycle 0
  Timer1.attachInterrupt(compute);  // Attach Timer1 interrupt to compute function
  TaskDisplayLCD();                 // Display initial values on LCD
  Timer1.start();                   // Start Timer1
}

int old; // Variable to store previous LED state

void loop() {
  // debounceButtons function call for debouncing button inputs
  debounceButtons();                // Debounce button inputs
  
  if (cong == 1) {
    interval2 = 1000 / square2Frequency;  // Calculate interval (milliseconds) for second square wave
    calculateTimes2();         // Calculate on and off times for second square wave
    TaskSquareWave2();        // Generate second square wave
  } else {
    digitalWrite(squareWave2Pin, LOW); // Set second square wave pin LOW
  }
  
  // Control delay pin based on mode
  if (chedo == 0) {
    digitalWrite(delayPin, HIGH); // Set delay pin HIGH if mode is 0
  } else {
    digitalWrite(delayPin, LOW);  // Set delay pin LOW if mode is not 0
  }
  
  // Toggle mode if button pressed
  if (digitalRead(ok) == 0) {
    cong = !cong;                  // Toggle mode
    TaskDisplayLCD();             // Update LCD display
    delay(300);                   // Debounce delay
  }
}

void debounceButtons() {
  // Increase frequency of square wave 1
  if (digitalRead(nutFreqSquareTang) == LOW) {
    xungxuong += 1; // Increase frequency
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Decrease frequency of square wave 1
  if (digitalRead(nutFreqSquareGiam) == LOW) {
    xungxuong -= 1; // Decrease frequency
    if (xungxuong < 1) xungxuong = 1; // Limit frequency
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Increase duty cycle of square wave 1
  if (digitalRead(nutDutySquareTang) == LOW) {
    xungnghi += 1; // Increase duty cycle
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Decrease duty cycle of square wave 1
  if (digitalRead(nutDutySquareGiam) == LOW) {
    xungnghi -= 1; // Decrease duty cycle
    if (xungnghi < 0) xungnghi = 0; // Limit duty cycle
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Increase frequency of second square wave
  if (digitalRead(tang_SX) == LOW) {
    soxung += 1; // Increase frequency
    if (square2Frequency > 100) square2Frequency = 100; // Limit frequency
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Decrease frequency of second square wave
  if (digitalRead(giam_SX) == LOW) {
    soxung -= 1; // Decrease frequency
    if (square2Frequency < 50) square2Frequency = 50; // Limit frequency
    TaskDisplayLCD(); // Update LCD display
    delay(debounceDelay); // Debounce delay
  }
  
  // Increase sine wave frequency
  if (digitalRead(nutFreqSineTang) == LOW) {
    sineFrequency += 1; // Increase frequency
    square2Frequency = sineFrequency; // Update second square wave frequency
    if (sineFrequency > 1000) sineFrequency = 1000; // Limit frequency
    TaskDisplayLCD(); // Update LCD display
    Timer1.stop(); // Stop Timer1
    // Update values for sine wave
    c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
    a[1] = A * sin(2 * pi * sineFrequency * T / 1000000.0);
    a[0] = 0.0;
    a[2] = 0.0;
    Timer1.start(); // Restart Timer1
    delay(debounceDelay); // Debounce delay
  }
  
  // Decrease sine wave frequency
  if (digitalRead(nutFreqSineGiam) == LOW) {
    sineFrequency -= 1; // Decrease frequency
    square2Frequency = sineFrequency; // Update second square wave frequency
    if (sineFrequency < 1) sineFrequency = 1; // Limit frequency
    TaskDisplayLCD(); // Update LCD display
    Timer1.stop(); // Stop Timer1
    // Update values for sine wave
    c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
    a[1] = A * sin(2 * pi * sineFrequency * T / 1000000.0);
    a[0] = 0.0;
    a[2] = 0.0;
    Timer1.start(); // Restart Timer1
    delay(debounceDelay); // Debounce delay
  }
}

// Task to generate square wave on pin 9 (Timer2)
void TaskSquareWave2() {
  if (j == 0) {
    unsigned long currentMillis = millis();  // Get current time
    if (ledState2 == LOW && currentMillis - previousMillis2 >= offTime2) {
      // Turn on LED
      previousMillis2 = currentMillis;
      digitalWrite(squareWave2Pin, HIGH);
      
      if (ledState2 != old) {
        h = h + 1; 
      }
      ledState2 = HIGH;
      old = ledState2;
    }
    else if (ledState2 == HIGH && currentMillis - previousMillis2 >= onTime2) {
      // Turn off LED
      previousMillis2 = currentMillis;
      digitalWrite(squareWave2Pin, LOW);
      ledState2 = LOW;
    }
  }  
  if (h >= soxung - 1) {
    L = 1;
    h = 0;
    j = 1;
    digitalWrite(squareWave2Pin, LOW); 
    ledState2 = HIGH;
  }      
  /*
  Serial.print(isrTime - time2); Serial.print("   "); 
  Serial.print(time2); Serial.print("   "); 
  Serial.print(isrTime); Serial.println("   "); */
  if (L == 1) {
    k = k + 1;
    if (h == xungnghi) {
      // Timer1.start();
      L = 0;
      h = 0;
    }
  }
  if (k >= 2) { 
    digitalWrite(xung_CKP, LOW);
    k = 0;
  }
  if (h >= xungxuong) {
    digitalWrite(xung_CKP, HIGH);
  }
  if (h < soxung && L == 0) { 
    j = 0;
  }
}

// Interrupt service routine for computing sine wave values
void compute() {
  if (cong == 0) {
    a[2] = c1 * a[1] - a[0];  // Recursive formula for sine wave
    a[0] = a[1];
    a[1] = a[2];
    if (a[2] >= 487.0) {  // Use a small offset to detect peak
      h = h + 0.315;
    }   
    // Serial.println(a[2]);
    // Serial.println(h);
    if (h >= soxung && L == 0) { 
      // Serial.println(h);
      L = 1;
      h = 0.0;
      k = k + 1;
      Timer1.setPwmDuty(pwmOutputPin, 0);  // Update PWM value
    }      
    /*
    Serial.print(isrTime - time2); Serial.print("   "); 
    Serial.print(time2); Serial.print("   "); 
    Serial.print(isrTime); Serial.println("   "); */
    if (L == 1) {
      if (h > xungnghi) {
        // Timer1.start();
        L = 0;
        h = 0.0;
      }
    }
    if (k >= 2) { 
      digitalWrite(xung_CKP, LOW);
      // Serial.println(k);
      k = 0;
    }
    if (h >= xungxuong - (xungnghi + 1)) {
      digitalWrite(xung_CKP, HIGH);
    }
    if (h < soxung && L == 0) { 
      Timer1.setPwmDuty(pwmOutputPin, int(a[2]) + 512);  // Update PWM value
    }
  }
}

// Task to display parameters on LCD
void TaskDisplayLCD() {
  lcd.setCursor(0, 3); // Set cursor to row 3, column 0
  lcd.print("CMP: ");
  lcd.print(xungxuong, 0); // Display pulse width
  lcd.print("  ");

  lcd.setCursor(0, 2); // Set cursor to row 2, column 0
  lcd.print("skeptical: ");
  lcd.print(xungnghi, 0); // Display duty cycle
  lcd.print("  ");

  lcd.setCursor(0, 0); // Set cursor to row 0, column 0
  lcd.print("Freq:  ");
  lcd.print(sineFrequency, 0); // Display sine wave frequency
  lcd.print(" Hz   ");

  lcd.setCursor(0, 1); // Set cursor to row 1, column 0
  lcd.print("CKP: ");
  lcd.print(soxung, 0); // Display number of pulses
  lcd.print(" "); 
  if (cong == 0) {
    lcd.print("SINE     "); // Display mode
    lcd.print(" ");
  } else {
    lcd.print("Sq/Trig  "); // Display mode
    lcd.print(" ");
  }
}

// Calculate timings for second square wave
void calculateTimes2() {
  interval2 = 1000 / square2Frequency;  // Calculate interval (milliseconds) for second square wave
  onTime2 = (interval2 * square2DutyCycle) / 100;  // Calculate ON time for second square wave
  offTime2 = interval2 - onTime2;            // Calculate OFF time for second square wave
}
