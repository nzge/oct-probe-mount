//======================================//
// For Arduino Uno:


// For Timer0 PWM PinA : Pin6  (OC0A)
// For Timer0 PWM PinB : Pin5  (OC0B)
// For Timer1 PWM PinA : Pin9  (OC1A)
// For Timer1 PWM PinB : Pin10 (OC1B)
// For Timer2 PWM PinA : Pin3  (OC2B)
// For Timer2 PWM PinB : Pin11 (OC2A)

//Pin Change Interrupt : Pin 2 and Pin 3
//======================================//
#define TimerFrequency 1000 // Hz
#define MotorRotateCW  // Comment out to rotate CCW
#define EncoderCountCW // Comment out to count encoder CCW
#define MotorEnablePin 6

#ifdef EncoderCountCW
const byte interruptPin1 = A2;
const byte interruptPin2 = A3;
#else
const byte interruptPin1 = A3;
const byte interruptPin2 = A2;
#endif

#define LimitSwitchPin 2

unsigned char EncoderState = 0; // 0~3
unsigned char LastEncoderState = 0;
long EncoderCounter = 0;
long DesiredPos = 0;
long Degree = 0;
/////
double kp = 2.5;
double ki = 0;//0.00005;
double kd = 0;
long AccumulatedErr = 0;
long LastErr = 0;
//
unsigned char PA,PB;
double output;

/*
 * 00 : 0
 * 01 : 1
 * 10 : 2
 * 11 : 3
 */

 /*
  * 00
  * 01
  * 11
  * 10
  */

void setup() {
  // put your setup code here, to run once:
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(MotorEnablePin, OUTPUT);
  pinMode(LimitSwitchPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin1), ReadEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), ReadEncoder, CHANGE);
  digitalWrite(MotorEnablePin,LOW);
  //
  Serial.begin(115200);
  //

  PCICR = 0b0000010; // Set PCIE1 = HIGH
  PCMSK1 = 0b00001100; // Set PCINT11 and PCINT10 = HIGH (Pin A3 and A2)
  //
  //Set Timer 1 for interrupt and PWM
  //Set WGM13-WGM10 : 0b1000 to use Mode 8 : PWM, phase and frequency correct
  //Set CS12 - CS10 to be 0b010 : the presclare is 8 
#ifdef MotorRotateCW
  TCCR1A = 0b10110000;  //COMxA1 COMxA0 COMxB1 COMxB0 - - WGMx1 WGMx0    x = 1
#else
  TCCR1A = 0b11100000;  //COMxA1 COMxA0 COMxB1 COMxB0 - - WGMx1 WGMx0    x = 1
#endif
  TCCR1B = 0b00010010;  //CNCx ICESx – WGMx3 WGMx2 CSx2 CSx1 CSx0
  ICR1 = 1000000/TimerFrequency;
  TIMSK1 = B00000001; // Enable Timer Interrupt
  OCR1A = ICR1/2;
  OCR1B = OCR1A;

  //Set Timer 2 for PWM
  //Set WGM22-WGM20 : 0b011 to use Mode 3 :  fast PWM
  //Set CS12 - CS10 to be 0b001 : the presclare is 1 
#ifdef MotorRotateCW
  TCCR2A = 0b11100011;  //COMxA1 COMxA0 COMxB1 COMxB0 - - WGMx1 WGMx0    x = 1
#else
  TCCR2A = 0b10110011;  //COMxA1 COMxA0 COMxB1 COMxB0 - - WGMx1 WGMx0    x = 1
#endif
  TCCR2B = 0b00000001;  //FOC2A FOC2B – – WGM22 CS22 CS21 CS20
  //TIMSK2 = B00000001; // Enable Timer Interrupt
  OCR2A = 127;
  OCR2B = OCR2A;
}

#define MAX_ENCODER_COUNTS 23122*2
#define FULL_ROTATION_DEGREES 360.0*2

int degreesToEncoderCounts(float degrees) {
    // Ensure the degrees are within the valid range
    if (degrees < 0) degrees = 0;

    // Calculate counts per degree
    float countsPerDegree = MAX_ENCODER_COUNTS / FULL_ROTATION_DEGREES;

    // Convert degrees to encoder counts
    return static_cast<int>(degrees * countsPerDegree + 0.5); // Adding 0.5 for rounding
}
void loop() {
  // put your main code here, to run repeatedly:

  Serial.print(PA);
  Serial.print("\t");
  Serial.print(PB);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");
  Serial.println(EncoderCounter);
  Serial.print("\t");
  Serial.print(Degree);

  Degree = Degree + 5;
  DesiredPos = degreesToEncoderCounts(Degree);
  
  //DesiredPos++;
  //DesiredPos = 23122;// 23122 per revolution
  //DesiredPos = long(23122)*long(0);
  //DesiredPos = 0;
  delay(100);
}


ISR(PCINT1_vect)  // Pin Change detected
{
  PA = digitalRead(interruptPin1);
  PB = digitalRead(interruptPin2);
  EncoderState = PA<<1 + PB;
  switch(LastEncoderState)
  {// 013201320132
    case 0:
      if(EncoderState==1) EncoderCounter++;
      else if(EncoderState==2) EncoderCounter--;
      break;
    case 1:   
      if(EncoderState==3) EncoderCounter++;
      else if(EncoderState==0) EncoderCounter--;
      break;
    case 2:
      if(EncoderState==0) EncoderCounter++;
      else if(EncoderState==3) EncoderCounter--;
      break;
    case 3:
      if(EncoderState==2) EncoderCounter++;
      else if(EncoderState==1) EncoderCounter--;
      break;
    default:
      break;
  }
  LastEncoderState = EncoderState;
}


ISR(TIMER1_OVF_vect)
{
  //OutputMoter(30);
  sei();
  MotorPosControlPID(DesiredPos);
}

int MotorPosControlPID(long pos)
{
  //sei();
  long err = pos - EncoderCounter;
  AccumulatedErr += err;
  long ErrDiff = err - LastErr;
  output = double(err) * kp + double(AccumulatedErr) * ki + double(ErrDiff) * kd;
  if(output>100) output = 100;
  else if(output<-100) output = -100;
  //
  //cli();
  OutputMoter(output);
  OutputMoter2(output);
  //sei();
  LastErr = err;
}

int OutputMoter(double Input) // put number between -100 ~ 100
{//Pin 9 and 10
  if(Input <= 1 && Input >= -1)
  {
    digitalWrite(MotorEnablePin,LOW);
  }
  else
  {
    digitalWrite(MotorEnablePin,HIGH);
  }
  OCR1A = long(Input*(double(ICR1/2))/101 + double(ICR1/2));
  OCR1B = OCR1A;
  return 1;
}

int OutputMoter2(double Input) // put number between -100 ~ 100
{//Pin 3 and 11
  if(Input <= 1 && Input >= -1)
  {
    digitalWrite(MotorEnablePin,LOW);
  }
  else
  {
    digitalWrite(MotorEnablePin,HIGH);
  }
  OCR2A = 127 + Input*127/100;
  OCR2B = OCR2A;
  return 1;
}
