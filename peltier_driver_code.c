#include <PID_v1.h>

//Porta ligada ao pino IN1 do modulo
int rele1 = 7;
//Porta ligada ao pino IN2 do modulo
int rele2 = 8;
//Porta ligada ao pino IN2 do modulo
int pwm = 9;
 
const int pinTemp = A0; //temp sensor pin (Input)

//PID parameters defaults 
//double kp=2;   //proportional parameter
//double ki=5;   //integral parameter
//double kd=1;   //derivative parameter

//Setpoint (temperature)
double Setpoint, Input, Output;
int room;

//Minimum and Maximum PWM command
//double commandMin = 0;
//double commandMax = 250;

//init PID
PID myPID(&Input, &Output, &Setpoint,2,0.5,0.5, DIRECT);

unsigned long serialTime; //this will help us know when to talk with processing

void setup()
{
  Serial.begin(9600);
  //Define pinos para o rele como saida
  pinMode(rele1, OUTPUT); 
  pinMode(rele2, OUTPUT);
  pinMode(pwm, OUTPUT);
  digitalWrite(rele1,LOW);
  digitalWrite(rele2,LOW);
  digitalWrite(pwm,LOW);
  room = 24;
  Setpoint = 10;
  myPID.SetMode(AUTOMATIC);
//  myPID.SetOutputLimits(commandMin, commandMax);
}

//get temp of pin sensor
double getTemp() {
  double temp;
  temp = (float(analogRead(pinTemp))*5/(1023))/0.01;
  return temp;
}

void loop (){
//Temp Read
  Input = getTemp();
  if(Setpoint >= room) {
  digitalWrite(rele1, LOW);
  digitalWrite(rele2, HIGH);
  myPID.SetControllerDirection(DIRECT);
  }
  else {
  digitalWrite(rele2, LOW);
  digitalWrite(rele1, HIGH);
  myPID.SetControllerDirection(REVERSE);
  }
  
  //process PID
  myPID.Compute();

  //apply PID processed command
  analogWrite(pwm, Output);

  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}


/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);   
  Serial.print(" ");
  Serial.print(Input);   
  Serial.print(" ");
  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(myPID.GetKp());   
  Serial.print(" ");
  Serial.print(myPID.GetKi());   
  Serial.print(" ");
  Serial.print(myPID.GetKd());   
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}


