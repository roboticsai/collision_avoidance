//a:
// Controller pins
int CH_1_PIN = A0;
int CH_2_PIN = A1;
// Motor driver pins
int adc_pin = A7;
int buzzer_pin = A2;
//original
const int APWM_PIN = 3;
const int BPWM_PIN = 11;
const int AIN1_PIN = 7;
const int AIN2_PIN = 9;
const int BIN1_PIN = 5;
const int BIN2_PIN = 4;
//FOR ADC/////////////////
const int adc_low_th = 385; //10.4Volts
const int adc_off_th =  370;
const int adc_ok_th = 424;
//for BUZZER//////////////
int buzzer_state = LOW;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 1000;           // interval at which to blink (milliseconds)
int PWMX = 0;
int PWMY = 0;
////////////////////////////
int low_voltage = 0;
int incomingByte = 0; // for incoming serial data


void setup() {
  // Configure pins
  Serial.begin(115200); // opens serial port, sets data rate to 9600 bps

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(APWM_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(BPWM_PIN, OUTPUT);
  // Enable motor driver

  pinMode(buzzer_pin, OUTPUT); //buzzer is active low
  digitalWrite(buzzer_pin, HIGH);
  TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
}

void toggle_buzzer() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (buzzer_state == LOW) {
      buzzer_state = HIGH;
    } else {
      buzzer_state = LOW;
    }
    digitalWrite(buzzer_pin, buzzer_state);
  }
}

void loop() {
  // Read pulse width from receiver
  int y = pulseIn(CH_2_PIN, HIGH,400);
  //  Serial.print("Y: \t");
  //  Serial.println(y);
  int x = pulseIn(CH_1_PIN, HIGH,400);
  //  Serial.print("-----------");
  //  Serial.print("X: \t");
  //  Serial.println(x);
  /*
    Serial.print(x);
    Serial.print("  ");
    Serial.println(y);
  */
  PWMX = pulseToPWM(x);
  PWMY = pulseToPWM(y);
//    Serial.print(PWMX);
//    Serial.println(PWMY);
//    delay(10);


  //////////////////For OBSTACLE AVOIDANCE/////////////
  ////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    char dummy = Serial.read();
    Serial.println(dummy);
    if (dummy == 'B') {
      //medium fAR
      analogWrite(APWM_PIN, 100);
      analogWrite(BPWM_PIN, 100);
      Serial.println("PWM 100");


    }

    else if (dummy == 'S') {
      //very near
      analogWrite(APWM_PIN, 0);
      analogWrite(BPWM_PIN, 0);
      Serial.println("PWM 0");


    }
    else {
      analogWrite(APWM_PIN, abs(PWMX));
      analogWrite(BPWM_PIN, abs(PWMY));
      Serial.println("NORMAL");
    }
  }

  


  //VMS///////////////////////////////
 // voltageManagement();
  ///////////////////////////////////


  if ( PWMX == 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
  } else if ( PWMX > 0 ) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
  } else {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
  }
  //////////////////////////////////
  // For Motor A
  ///////////////////////////////////
  if ( PWMY == 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
  } else if ( PWMY > 0 ) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
  } else {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }
}

///////////////////////////////////
// VMS Routine, checks ADC value
///////////////////////////////////
void voltageManagement() {
  int z = analogRead(adc_pin);
  // Before Mapping Values
  // 642 12.5V
  //769 14.5
  //  Serial.println(z);

  if (z < adc_low_th) {
    low_voltage = 1;
  }

  if (z < adc_off_th) {
    digitalWrite(buzzer_pin, LOW);

    PWMX = 0;
    PWMY = 0;
    analogWrite(APWM_PIN, abs(PWMX));
    analogWrite(BPWM_PIN, abs(PWMY));
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    //    while (z <= adc_ok_th) {
    //      z = analogRead(adc_pin);
    //      Serial.println(z);
    //
    //      PWMX = 0;
    //      PWMY = 0;
    //      analogWrite(APWM_PIN, abs(PWMX));
    //      analogWrite(BPWM_PIN, abs(PWMY));
    //      digitalWrite(AIN1_PIN, HIGH);
    //      digitalWrite(AIN2_PIN, HIGH);
    //
    //    }
    low_voltage = 0;
  }

  else {
    low_voltage = 0;
    digitalWrite(buzzer_pin, HIGH);
  }


  if (low_voltage) {
    toggle_buzzer();
  }
}

int pulseToPWM(int pwm)
{
  if (pwm > 970) {
    pwm = map(pwm, 970, 1989, -255, 255);
  }
  else {
    pwm = 0;
  }
  if ( abs(pwm) <= 10) {
    pwm = 0;
  }
  return pwm;
}
