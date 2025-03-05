#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Định nghĩa các chân nút nhấn
const int nutFreqSquareTang = 2;
const int nutFreqSquareGiam = 3;
const int nutDutySquareTang = 4;
const int nutDutySquareGiam = 6;
const int nutFreqSquare2Tang = 7;
const int nutFreqSquare2Giam = 8;
const int nutFreqSineTang = A1;
const int nutFreqSineGiam = A2;

// Định nghĩa các chân output
const int squareWavePin = 5;  // Timer0
const int squareWave2Pin = 9;  // Timer2
const int pwmOutputPin = 10;
const int delayPin = 11;

// Khởi tạo màn hình LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Các thông số cho sóng vuông
volatile int squareFrequency = 50;  // Tần số sóng vuông ban đầu (Hz)
volatile int squareDutyCycle = 50;   // Chu kỳ làm việc ban đầu (%)

// Các thông số cho sóng vuông thứ hai
volatile int square2Frequency = 50; // Tần số sóng vuông thứ hai ban đầu (Hz)
volatile int square2DutyCycle = 10; // Chu kỳ làm việc cho sóng vuông thứ hai (%)

// Các thông số cho thuật toán sine
const float pi = 3.14159;
volatile float T = 100;    // Thời gian mẫu trong micro giây
volatile float sineFrequency = 50;  // Tần số sine
const float A = 490;    // Biên độ
float a[] = {0.0, A * sin(2 * pi * sineFrequency * T / 1000000.0), 0.0};
volatile float c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
int debounceDelay = 50;

unsigned long previousMillis1 = 0;  // Thời gian trước đó cho sóng vuông 1
unsigned long interval1;            // Khoảng thời gian giữa các xung cho sóng vuông 1
unsigned long onTime1;              // Thời gian bật đèn LED cho sóng vuông 1
unsigned long offTime1;             // Thời gian tắt đèn LED cho sóng vuông 1
bool ledState1 = LOW;               // Trạng thái của đèn LED 1

unsigned long previousMillis2 = 0;  // Thời gian trước đó cho sóng vuông 2
unsigned long interval2;            // Khoảng thời gian giữa các xung cho sóng vuông 2
unsigned long onTime2;              // Thời gian bật đèn LED cho sóng vuông 2
unsigned long offTime2;             // Thời gian tắt đèn LED cho sóng vuông 2
bool ledState2 = LOW;               // Trạng thái của đèn LED 2

void setup() {
  // Khởi tạo các chân input
  pinMode(nutFreqSquareTang, INPUT_PULLUP);
  pinMode(nutFreqSquareGiam, INPUT_PULLUP);
  pinMode(nutDutySquareTang, INPUT_PULLUP);
  pinMode(nutDutySquareGiam, INPUT_PULLUP);
  pinMode(nutFreqSquare2Tang, INPUT_PULLUP);
  pinMode(nutFreqSquare2Giam, INPUT_PULLUP);
  pinMode(nutFreqSineTang, INPUT_PULLUP);
  pinMode(nutFreqSineGiam, INPUT_PULLUP);

  // Khởi tạo các chân output
  pinMode(squareWavePin, OUTPUT);
  pinMode(squareWave2Pin, OUTPUT);
  pinMode(pwmOutputPin, OUTPUT);
  pinMode(delayPin, OUTPUT);
  
  // Khởi tạo màn hình LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.begin(9600);
  // Khởi tạo Timer1 cho sóng sine
  Timer1.initialize(T);  // Thiết lập thời gian mẫu cho tín hiệu
  Timer1.pwm(pwmOutputPin, 0, T);  // Tạo tín hiệu PWM trên outPin(10) với chu kỳ 0 và thời gian mẫu T=100us
  Timer1.attachInterrupt(compute);  // Gắn ngắt Timer1 với hàm compute

  TaskDisplayLCD();
}

void loop() {
  debounceButtons();
  interval1 = 1000 / squareFrequency;  // Tính chu kỳ (millisecond) cho sóng vuông 1
  calculateTimes1();         // Tính toán thời gian bật và tắt cho sóng vuông 1
  TaskSquareWave1();

  interval2 = 1000 / square2Frequency;  // Tính chu kỳ (millisecond) cho sóng vuông 2
  calculateTimes2();         // Tính toán thời gian bật và tắt cho sóng vuông 2
  TaskSquareWave2(); 
}

void debounceButtons() {
  if (digitalRead(nutFreqSquareTang) == LOW) {
    squareFrequency += 1; // Tăng tần số sóng vuông 1
    if (squareFrequency > 1000) squareFrequency = 1000; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSquareGiam) == LOW) {
    squareFrequency -= 1; // Giảm tần số sóng vuông 1
    if (squareFrequency < 1) squareFrequency = 1; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutDutySquareTang) == LOW) {
    squareDutyCycle += 1; // Tăng chu kỳ làm việc sóng vuông 1
    if (squareDutyCycle > 100) squareDutyCycle = 100; // Giới hạn chu kỳ làm việc
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutDutySquareGiam) == LOW) {
    squareDutyCycle -= 1; // Giảm chu kỳ làm việc sóng vuông 1
    if (squareDutyCycle < 0) squareDutyCycle = 0; // Giới hạn chu kỳ làm việc
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSquare2Tang) == LOW) {
    square2Frequency += 1; // Tăng tần số sóng vuông thứ hai
    if (square2Frequency > 100) square2Frequency = 100; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSquare2Giam) == LOW) {
    square2Frequency -= 1; // Giảm tần số sóng vuông thứ hai
    if (square2Frequency < 50) square2Frequency = 50; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSineTang) == LOW) {
    sineFrequency += 1; // Tăng tần số sine
    if (sineFrequency > 1000) sineFrequency = 1000; // Giới hạn tần số
    TaskDisplayLCD();
    Timer1.stop();
    // Cập nhật các giá trị cho sóng sine
    c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
    a[1] = A * sin(2 * pi * sineFrequency * T / 1000000.0);
    a[0] = 0.0;
    a[2] = 0.0;
    // Khởi động lại Timer1
    Timer1.start(); 
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSineGiam) == LOW) {
    sineFrequency -= 1; // Giảm tần số sine
    if (sineFrequency < 1) sineFrequency = 1; // Giới hạn tần số
    TaskDisplayLCD();
    Timer1.stop();
    // Cập nhật các giá trị cho sóng sine
    c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
    a[1] = A * sin(2 * pi * sineFrequency * T / 1000000.0);
    a[0] = 0.0;
    a[2] = 0.0;
    // Khởi động lại Timer1
    Timer1.start();
    delay(debounceDelay);
  }
}

// Nhiệm vụ tạo sóng vuông trên chân 5 (Timer0)
void TaskSquareWave1() {
  unsigned long currentMillis = millis();  // Lấy thời gian hiện tại

  if (ledState1 == LOW && currentMillis - previousMillis1 >= offTime1) {
    // Bật đèn LED
    previousMillis1 = currentMillis;
    digitalWrite(squareWavePin, HIGH);
    ledState1 = HIGH;
  } else if (ledState1 == HIGH && currentMillis - previousMillis1 >= onTime1) {
    // Tắt đèn LED
    previousMillis1 = currentMillis;
    digitalWrite(squareWavePin, LOW);
    ledState1 = LOW;
  }
}

// Nhiệm vụ tạo sóng vuông trên chân 9 (Timer2)
void TaskSquareWave2() {
  unsigned long currentMillis = millis();  // Lấy thời gian hiện tại

  if (ledState2 == LOW && currentMillis - previousMillis2 >= offTime2) {
    // Bật đèn LED
    previousMillis2 = currentMillis;
    digitalWrite(squareWave2Pin, HIGH);
    ledState2 = HIGH;
  } else if (ledState2 == HIGH && currentMillis - previousMillis2 >= onTime2) {
    // Tắt đèn LED
    previousMillis2 = currentMillis;
    digitalWrite(squareWave2Pin, LOW);
    ledState2 = LOW;
  }
}

// Hàm xử lý ngắt để tính toán giá trị sóng sine
void compute() {
  digitalWrite(delayPin, HIGH);
  a[2] = c1 * a[1] - a[0];  // Công thức đệ quy
  a[0] = a[1];
  a[1] = a[2];
  Timer1.setPwmDuty(pwmOutputPin, int(a[2]) + 512);  // Cập nhật giá trị PWM
  digitalWrite(delayPin, LOW);
}

// Nhiệm vụ hiển thị thông số lên LCD
void TaskDisplayLCD() {
  lcd.setCursor(0, 0);
  lcd.print("Square Freq: ");
  lcd.print(squareFrequency);
  lcd.print(" Hz   ");

  lcd.setCursor(0, 1);
  lcd.print("Square Duty: ");
  lcd.print(squareDutyCycle);
  lcd.print(" %   ");

  lcd.setCursor(0, 2);
  lcd.print("Sine Freq: ");
  lcd.print(sineFrequency, 0);
  lcd.print(" Hz   ");

  lcd.setCursor(0, 3);
  lcd.print("Triangle: ");
  lcd.print(square2Frequency);
  lcd.print(" Hz   ");
}

void calculateTimes1() {
  interval1 = 1000 / squareFrequency;  // Tính chu kỳ (millisecond) cho sóng vuông 1
  onTime1 = (interval1 * squareDutyCycle) / 100;  // Tính thời gian bật đèn LED cho sóng vuông 1
  offTime1 = interval1 - onTime1;            // Tính thời gian tắt đèn LED cho sóng vuông 1
}

void calculateTimes2() {
  interval2 = 1000 / square2Frequency;  // Tính chu kỳ (millisecond) cho sóng vuông 2
  onTime2 = (interval2 * square2DutyCycle) / 100;  // Tính thời gian bật đèn LED cho sóng vuông 2
  offTime2 = interval2 - onTime2;            // Tính thời gian tắt đèn LED cho sóng vuông 2
}
