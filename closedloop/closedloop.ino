/*

  Powerflow_direction   1 - 2       0
                        2 - 1       1

*/


int Active_PWM  = 2;//T3
int Sync_PWM    = 5;//T3
int High_Switch = 6; //T4
int Low_Switch  = 7;//T4

int S1 = 0; // Duty of switch1
int S2 = 0; // Duty of switch2
int S3 = 0; // Duty of switch3
int S4 = 0; // Duty of switch4


int Powerflow_Direction = 0; // 1
bool mode = true;

//input
#define Battery_1 A1
#define Battery_2 A2
#define Inductor_current A0


//Battery voltage
 
// Floats for ADC voltage & Input voltage
float adc_voltage_1 = 0.0;
float in_voltage_1 = 0.0;
float adc_voltage_2 = 0.0;
float in_voltage_2 = 0.0;
 
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
 
// Float for Reference Voltage
float ref_voltage = 5.0;
 
// Integer for ADC value
int adc_value_1 = 0;
int adc_value_2 = 0;

//current sensor 

int sensitivity = 66;
int Inductor_adcValue = 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;

//PI Controller

double Setpoint;
double Input;
double Output = 0;

double Kp = 0,Ki = 0,Kd = 0;

double error = 0;
double error_1 = 0;
double out = 0;

int c;
void setup()
{
   pinMode(Active_PWM, OUTPUT); //1
   pinMode(Sync_PWM, OUTPUT); ///2
   pinMode(High_Switch,OUTPUT);//3
   pinMode(Low_Switch,OUTPUT);//4

  PI_set();
  

  TCCR3A = 0; 
  TCCR3B = 0;
  TCNT3 = 0;

  TCCR3B |= _BV(CS30); 
  ICR3 = 160;

  //OCR3A = 19;//81;//19,86 deadtime

  TCCR3A |= _BV(COM3A1) | _BV(COM3A0); 

  //OCR3B = 13;//79; //13
 
  TCCR3A |= _BV(COM3B1); 

  TCCR3B |= _BV(WGM33); 
  TCCR3A |= _BV(WGM31); 
  
  TCCR4A = 0; 
  TCCR4B = 0;
  TCNT4 = 0;


  TCCR4B |= _BV(CS40); 
  ICR4 = 160;

  //OCR4A = 81;//19;//81,86 deadtime
 
  TCCR4A |= _BV(COM4A1) | _BV(COM4A0); 

  //OCR4B = 79;//13; //79
 
  TCCR4A |= _BV(COM4B1); 

  TCCR4B |= _BV(WGM43); 
  TCCR4A |= _BV(WGM41);
  
   // Setup Serial Monitor
   Serial.begin(1000000);
   Serial.println("DC Voltage Test");
}

void loop(){
    Mode_set();
    //PID_control();
    double PI_out =Calc_PI(Inductor_current_measure(),(micros()* 1e-6));//Inductor_current_measure());   
    static int pwm = 81;
    PI_out = PI_out/10000000;
    Serial.println(PI_out);
    PI_out += 80;    
    if(PI_out > 160)
      PI_out = 150;
    else if(PI_out < 0)
      PI_out = 10;
    pwm = PI_out;
    Serial.print("PWM : ");      
    Serial.println(PI_out);
}

void PI_set()
{
  if(Powerflow_Direction == 0)
   {
     Ki = 247686.81;
     Kp = 4.1624e-6;
     Setpoint = 4.1667;
   }
   else
   {
     Ki = 247686.81;
     Kp = 4.1624e-6;;
     Setpoint = -4.1667;
   }
}

double Calc_PI(double input,float time)
{
  static float integral_error = 0;
  static float last_time = time;  
  float delta_time = time - last_time;
  error = Setpoint - input;
  integral_error += error * delta_time;
  Output = Output + Kp*error + Ki*(integral_error);
  last_time = time;
  return Output;
}

/*
void PID_control()
{
    Input = Inductor_current_measure();
    Serial.print("Inductor current = ");
    Serial.println(Inductor_current_measure(), 2);
    Serial.print("OUTPUT = ");
    Serial.println(OUTPUT, 2);

}*/

void Mode_set()
{
  if(Powerflow_Direction == 0)
  {
    if(Battery_voltage_1() > Battery_voltage_2())
    {
      S1 = 81;
      S2 = 79;
      S3 = 19;
      S4 = 13;
      Serial.print("Input Voltage1 = ");
      Serial.println(Battery_voltage_1(), 2);
      Serial.print("Input Voltage2 = ");
      Serial.println(Battery_voltage_2(), 2);
      Serial.println("Buck12");
    }
    else
    {
      S1 = 19;
      S2 = 13;
      S3 = 81;
      S4 = 79;
      Serial.print("Input Voltage1 = ");
      Serial.println(Battery_voltage_1(), 2);
      Serial.print("Input Voltage2 = ");
      Serial.println(Battery_voltage_2(), 2); 
      Serial.println("Boost12");
    }
  }
  else
  {
    if(Battery_voltage_1() > Battery_voltage_2())
    {
      S1 = 81;
      S2 = 79;
      S3 = 19;
      S4 = 13;
      Serial.print("Input Voltage1 = ");
      Serial.println(Battery_voltage_1(), 2);
      Serial.print("Input Voltage2 = ");
      Serial.println(Battery_voltage_2(), 2);
      Serial.println("Boost21");
    }
    else
    {
       S1 = 19;
       S2 = 13;
       S3 = 81;
       S4 = 79;
      Serial.print("Input Voltage1 = ");
      Serial.println(Battery_voltage_1(), 2);
      Serial.print("Input Voltage2 = ");
      Serial.println(Battery_voltage_2(), 2);
      Serial.println("Buck21"); 
    }    
  }
  Duty_cycle(S1,S2,S3,S4);
  delay(500);
}

void Duty_cycle(int a,int b,int c,int d)
{
  OCR3A = a;//19;
  OCR3B = b;//13;
  OCR4A = c;//81;
  OCR4B = d;//79;

}

float Battery_voltage_1()
{
  // Read the Analog Input
   adc_value_1 = analogRead(Battery_1);
  // Determine voltage at ADC input
   adc_voltage_1  = (adc_value_1 * ref_voltage) / 1024.0;
  // Calculate voltage at divider input
   in_voltage_1 = adc_voltage_1 / (R2/(R1+R2)) ;
  return in_voltage_1;   
}

float Battery_voltage_2()
{
  adc_value_2 = analogRead(Battery_2);
  adc_voltage_2  = (adc_value_2 * ref_voltage) / 1024.0; 
  in_voltage_2 = adc_voltage_2 / (R2/(R1+R2)) ;
  return in_voltage_2;
}

double Inductor_current_measure()
{
  Inductor_adcValue = analogRead(Inductor_current);
  adcVoltage = (Inductor_adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  return 4.36687;//currentValue;
}