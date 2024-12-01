#include <TimerOne.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Định nghĩa các chân nút nhấn
const int nutFreqSquareTang = 2;
const int nutFreqSquareGiam = 3;
const int nutDutySquareTang = 4;
const int nutDutySquareGiam = 6;
const int tang_SX = 7;
const int giam_SX = 8;
const int nutFreqSineTang = A1;
const int nutFreqSineGiam = A2; 
const int ok = 12;
// Định nghĩa các chân output
const int xung_CKP = 5;  // Timer0
const int squareWave2Pin = 9;  // Timer2
const int pwmOutputPin = 10;
const int delayPin = 11;

// Khởi tạo màn hình LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);


// Các thông số cho thuật toán sine
const float pi = 3.14159;
volatile float T = 100;    // Thời gian mẫu trong micro giây
volatile float sineFrequency = 100;  // Tần số sine
const float A = 490;    // Biên độ
float a[] = {0.0, A * sin(2 * pi * sineFrequency * T / 1000000.0), 0.0};
volatile float c1 = (8.0 - 2.0 * pow(2 * pi * sineFrequency * T / 1000000.0, 2)) / (4.0 + pow(2 * pi * sineFrequency * T / 1000000.0, 2));
int debounceDelay = 50;
// Các thông số cho sóng vuông thứ hai
volatile int square2Frequency = sineFrequency; // Tần số sóng vuông thứ hai ban đầu (Hz)
volatile int square2DutyCycle = 10; // Chu kỳ làm việc cho sóng vuông thứ hai (%)

unsigned long previousMillis2 = 0;  // Thời gian trước đó cho sóng vuông 2
unsigned long interval2;            // Khoảng thời gian giữa các xung cho sóng vuông 2
unsigned long onTime2;              // Thời gian bật đèn LED cho sóng vuông 2
unsigned long offTime2;             // Thời gian tắt đèn LED cho sóng vuông 2
bool ledState2 = LOW;               // Trạng thái của đèn LED 2
unsigned long time1;
unsigned long time2;
float soxung = 20.0;
float xungxuong = 5.0;
float xungnghi = 2.0;
int j = 0,L=0;
float i=0.0,h=0.0;
int cong =0;
float k = 0;
bool chedo = false;
volatile unsigned long isrTime;
void setup() {
  // Khởi tạo các chân input
  square2Frequency = sineFrequency;
  pinMode(nutFreqSquareTang, INPUT_PULLUP);
  pinMode(nutFreqSquareGiam, INPUT_PULLUP);
  pinMode(nutDutySquareTang, INPUT_PULLUP);
  pinMode(nutDutySquareGiam, INPUT_PULLUP);
  pinMode(tang_SX, INPUT_PULLUP);
  pinMode(giam_SX, INPUT_PULLUP);
  pinMode(nutFreqSineTang, INPUT_PULLUP);
  pinMode(nutFreqSineGiam, INPUT_PULLUP); 
  pinMode(ok, INPUT_PULLUP);
  // Khởi tạo các chân output
  pinMode(xung_CKP, OUTPUT);
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
  Timer1.start();
  //lcd.clear();
}
int old;
void loop() {
  /*isrTime = millis();
  Serial.print(millis());
  Serial.print("   ");
  Serial.println(isrTime);*/   
  debounceButtons();
  if(cong == 1 ){
  interval2 = 1000 / square2Frequency;  // Tính chu kỳ (millisecond) cho sóng vuông 2
  calculateTimes2();         // Tính toán thời gian bật và tắt cho sóng vuông 2
  TaskSquareWave2();    
  }
  else digitalWrite(squareWave2Pin, LOW);
 
  if(chedo == 0){
    digitalWrite(delayPin, HIGH);
  }
  else digitalWrite(delayPin, LOW);
  if(digitalRead(ok) == 0){
    cong = !cong;
     TaskDisplayLCD(); 
    delay(300); 
  }
  
   
}

void debounceButtons() {
  if (digitalRead(nutFreqSquareTang) == LOW) {
    xungxuong += 1; // Tăng tần số sóng vuông 1
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSquareGiam) == LOW) {
    xungxuong -= 1; // Giảm tần số sóng vuông 1
    if (xungxuong < 1) xungxuong = 1; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutDutySquareTang) == LOW) {
    xungnghi += 1; // Tăng chu kỳ làm việc sóng vuông 1
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutDutySquareGiam) == LOW) {
    xungnghi -= 1; // Giảm chu kỳ làm việc sóng vuông 1
    if (xungnghi < 0) xungnghi = 0; // Giới hạn chu kỳ làm việc
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(tang_SX) == LOW) {
    soxung += 1; // Tăng tần số sóng vuông thứ hai
    if (square2Frequency > 100) square2Frequency = 100; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(giam_SX) == LOW) {
    soxung -= 1; // Giảm tần số sóng vuông thứ hai
    if (square2Frequency < 50) square2Frequency = 50; // Giới hạn tần số
    TaskDisplayLCD();
    delay(debounceDelay);
  }
  if (digitalRead(nutFreqSineTang) == LOW) {
    sineFrequency += 1; // Tăng tần số sine
    square2Frequency = sineFrequency;
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
    square2Frequency = sineFrequency;
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


// Nhiệm vụ tạo sóng vuông trên chân 9 (Timer2)
void TaskSquareWave2() {
  if(j == 0){ 
    unsigned long currentMillis = millis();  // Lấy thời gian hiện tại
  if (ledState2 == LOW && currentMillis - previousMillis2 >= offTime2) {
    // Bật đèn LED
    previousMillis2 = currentMillis;
    digitalWrite(squareWave2Pin, HIGH);
    
    if(ledState2 != old){
      h = h+1; 
    }
    ledState2 = HIGH;
    old =  ledState2;
    
  }
  else if (ledState2 == HIGH && currentMillis - previousMillis2 >= onTime2) {
    // Tắt đèn LED
    previousMillis2 = currentMillis;
    digitalWrite(squareWave2Pin, LOW);
    ledState2 = LOW;
  }
  }  
  if(h >= soxung - 1){
    L = 1;
    h = 0;
    j = 1;
    digitalWrite(squareWave2Pin, LOW); 
    ledState2 = HIGH;
   }      
   /*
   Serial.print(isrTime - time2);Serial.print("   "); 
   Serial.print(time2);Serial.print("   "); 
   Serial.print(isrTime);Serial.println("   "); */
   if(L == 1){
    k = k+1;
    if(h == xungnghi){
    //Timer1.start();
    L = 0;
    h = 0;
    }
   }
   if(k >= 2){ 
   digitalWrite(xung_CKP, LOW);
   k = 0;
   }
   if(h >= xungxuong){
   digitalWrite(xung_CKP, HIGH);
   
  }
   if(h < soxung && L == 0){ 
  j = 0;
   }
}

 
// Hàm xử lý ngắt để tính toán giá trị sóng sine
void compute() {
  if(cong == 0){
 a[2] = c1 * a[1] - a[0];  // Công thức đệ quy
  a[0] = a[1];
  a[1] = a[2]; 
  if (a[2] >= 487.0) {  // Sử dụng độ lệch nhỏ để xác định đỉnh
      h = h + 0.315;
    }   
    //Serial.println(a[2]);
   // Serial.println(h);
   if(h >= soxung  && L==0){ 
    //Serial.println(h);
    L = 1;
    h = 0.0;
    k = k+1;
    Timer1.setPwmDuty(pwmOutputPin, 0);  // Cập nhật giá trị PWM
   }      
   /*
   Serial.print(isrTime - time2);Serial.print("   "); 
   Serial.print(time2);Serial.print("   "); 
   Serial.print(isrTime);Serial.println("   "); */
   if(L == 1){
    if(h > xungnghi){
    //Timer1.start();
    L = 0;
    h = 0.0;
    }
   }
   if(k >= 2){ 
   digitalWrite(xung_CKP, LOW);
   //Serial.println(k);
   k = 0;
   }
   if(h >= xungxuong-(xungnghi+1)){
   digitalWrite(xung_CKP, HIGH);  
  }
   if(h  < soxung && L == 0){ 
  Timer1.setPwmDuty(pwmOutputPin, int(a[2]) + 512);  // Cập nhật giá trị PWM
   }
  }
}
// Nhiệm vụ hiển thị thông số lên LCD
void TaskDisplayLCD() {
  lcd.setCursor(0, 3);
  lcd.print("CMP: ");
  lcd.print(xungxuong,0);
  lcd.print("  ");

  lcd.setCursor(0, 2);
  lcd.print("skeptical: ");
  lcd.print(xungnghi,0);
  lcd.print("  ");

  lcd.setCursor(0, 0);
  lcd.print("Freq:  ");
  lcd.print(sineFrequency,0);
  lcd.print(" Hz   ");

  lcd.setCursor(0, 1);
  lcd.print("CKP: ");
  lcd.print(soxung,0);
  lcd.print(" "); 
  if(cong == 0){
  lcd.print("SINE     "); 
  lcd.print(" ");
  }
  else{
    lcd.print("Sq/Trig  "); 
  lcd.print(" ");
  }
}


void calculateTimes2() {
  interval2 = 1000 / square2Frequency;  // Tính chu kỳ (millisecond) cho sóng vuông 2
  onTime2 = (interval2 * square2DutyCycle) / 100;  // Tính thời gian bật đèn LED cho sóng vuông 2
  offTime2 = interval2 - onTime2;            // Tính thời gian tắt đèn LED cho sóng vuông 2
}
