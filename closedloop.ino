/*

  Powerflow_direction   1 - 2       0
                        2 - 1       1


 Mode                   1      2          3        4
                    Buck 12   Boost 12   Boost 21 Buck 21
*/

int mode;


int Active_PWM  = 2;//T3
int Sync_PWM    = 5;//T3
int High_Switch = 6; //T4
int Low_Switch  = 7;//T4

int S1 = 0; // Duty of switch1
int S2 = 0; // Duty of switch2
int S3 = 0; // Duty of switch3
int S4 = 0; // Duty of switch4


int Powerflow_Direction = 0; // 1


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

double I_ref;
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

  float bv_1,bv_2,ind_current;

  bv_1 = Battery_voltage_1();
  bv_2 = Battery_voltage_2();

  ind_current = Inductor_current_measure();

  print_voltage(bv_1,bv_2);
  print_current(ind_current);

  double Duty = pwm_set(ind_current); 
  Mode_sequence_set(Duty,bv_1,bv_2);    
}

// calculates the duty cycle using PI controller and returns the Active value

double pwm_set(float Ind_current)
{
    double PI_out =Calc_PI(Ind_current,(micros()* 1e-6));//Inductor_current_measure());   
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
    Serial.println(pwm);
    return pwm;
}

// sets the Kp,Ki and I_ref for PI controller based on the mode
void PI_set()
{
  if(Powerflow_Direction == 0)
   {
     Ki = 350005.50;
     Kp = 6.65014 * 1e-6;
     I_ref = 4.1667;
   }
   else
   {
     Ki = 247686.81;  //boost
     Kp = 4.1624 * 1e-6;;
     I_ref = -4.1667;
   }
}

//PI controller function

double Calc_PI(double I_L,float time)
{
  static float integral_error = 0;
  static float last_time = time;  
  float delta_time = time - last_time;
  error = I_ref - I_L;
  integral_error += error * delta_time;
  Output = Output + Kp*error + Ki*(integral_error);
  last_time = time;
  return Output;
}

//Returns the mode 

int mode_set(float bv_1,float bv_2)
{
  if(Powerflow_Direction == 0)
  {
    if(bv_1 > bv_2 )
    {
      mode = 1;      
    }
    else
    {
      mode = 2;
    }
  }
  else
  {
    if(bv_1 > bv_2 )
    {
      mode = 3;      
    }
    else
    {
      mode = 4;
    }
  }
  return mode;
}

//provide Duty ccle as per sequence table

void Mode_sequence_set(double duty,float bv_1,float bv_2)
{
  int Mode = mode_set(bv_1,bv_2);
   if(Mode == 1)
    {
      S1 = duty + 1;
      S2 = duty - 1;
      S3 = 19;
      S4 = 13;
      Serial.println("Buck12");
    }
    else if(Mode == 2)
    {
      S1 = 19;
      S2 = 13;
      S3 = duty + 1;
      S4 = duty - 1;
      Serial.println("Boost12");
    }
    else if(Mode == 3) 
    {
      S1 = duty + 1;
      S2 = duty - 1;
      S3 = 19;
      S4 = 13;
      Serial.println("Boost 21");
      
    }
    else if(Mode == 4)
    {
       S1 = 19;
       S2 = 13;
       S3 = duty + 1;
       S4 = duty - 1;
      Serial.println("Buck 21");
    }    
  Duty_cycle(S1,S2,S3,S4);
  delay(500);
}

// sets the duty cycle values to the registers

void Duty_cycle(int a,int b,int c,int d)
{
  OCR3A = a;//19;
  OCR3B = b;//13;
  OCR4A = c;//81;
  OCR4B = d;//79;

}

//calculates the battery voltage

float Battery_voltage_1()
{
   adc_value_1 = analogRead(Battery_1);
   adc_voltage_1  = (adc_value_1 * ref_voltage) / 1024.0;
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

//calculates inductor current

double Inductor_current_measure()
{
  Inductor_adcValue = analogRead(Inductor_current);
  adcVoltage = (Inductor_adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  return currentValue;
}

void print_current(float IL)
{
  Serial.print("Inductor current = ");
  Serial.println(IL,2);
}
void print_voltage(float bv_1,float bv_2)
{
  Serial.print("Input Voltage1 = ");
  Serial.println(bv_1, 2);
  Serial.print("Input Voltage2 = ");
  Serial.println(bv_2, 2);
}