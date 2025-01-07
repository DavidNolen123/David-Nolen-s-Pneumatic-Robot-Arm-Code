#include <Servo.h>

class Action {
  public:
    int stepTarget;
    double a1Angle;
    double a2Angle;
    double a3Angle;
    boolean clawState;

    int milliTime;

    public: Action(int stepTarget, double a1Angle, double a2Angle, double a3Angle, int milliTime, boolean clawState) {
        this->stepTarget = stepTarget;
        this->a1Angle = a1Angle;
        this->a2Angle = a2Angle;
        this->a3Angle = a3Angle;
        this->milliTime = milliTime;
        this->clawState = clawState;
    }
    public: int getStepTarget() {
        return stepTarget;
    }
    public: double geta1Angle() {
        return a1Angle;
    }
    public: double geta2Angle() {
        return a2Angle;
    }
    public: double geta3Angle() {
        return a3Angle;
    }
    public: double getmilliTime() {
        return milliTime;
    }
    public: double getclawState() {
        return clawState;
    }

};

int targetNum = 0;
Action home = Action (0, 80, 136, 90, 3000, false);//5
Action grabOut = Action(3000, 200, 70, 130, 3000, false);//5
Action grabThat = Action(3000, 200, 70, 130, 1000, true);//2
Action bringIt = Action(-2000, 80, 200, 0, 3000, true);
Action outAgain = Action(-2000, 100, 100, 50, 1000, true);//2
Action release = Action(-2000, 100, 100, 50, 2000, false);
//home


Action grabOut2 = Action(2500, 200, 70, 130, 5000, false);
Action grabThat2 = Action(2500, 200, 70, 130, 2000, true);

Action doTheStuffs[] = {home, grabOut, grabThat, bringIt, outAgain, release, grabOut2, grabThat2, bringIt, outAgain, release, home};

class Coordinate {
  public:
    double x;
    double y;

    public: Coordinate(double x, double y) {
        this->x = x;
        this->y = y;
    }
    public: double getX() {
        return x;
    }
    public: double getY() {
        return y;
    }
};

double angle1 = 0;
double angle2 = 0;
double angle3 = 0;
//all thisi eventually needs to go in the looop
//in inches
Coordinate pt1 = Coordinate(0, 5);

//Pt2 is the pivot point for the arm2

double arm1Len = 12;//distance between pivots
double arm2Len = 9;
double arm3Len = 5;//from servo joint to center of pincers

Coordinate pt2 = Coordinate(0,0);//equation to calculate pt2

//Pt3 is the servo pivot point, pivot for arm3
//origin relative angle2:
double orgRelAng2 = 0;//later: angle2 + angle1
Coordinate pt3 = Coordinate(0,0);

//origin relative angle3:
double orgRelAng3 = orgRelAng2 + 0;//(+ angle3)
Coordinate pt4 = Coordinate(0,0);
//need future program for calculating joint coordinates (in 2d exc stepper)


boolean test = home.getclawState();

String incomingByte; // for incoming serial data

int currentEncoderTime;
int lastEncoderTime;

int stepPos = 0; //the initialized pos params
int stepTargetPos = 0;
double servoTargetAngle = 0;
double targetA1 = 0;
double targetA2 = 0;
double targetA3 = 0;
double clawOn = false;
double waitTime = 10000; //otherwise it will continue imediently before waitime is actually initialized

//PID algorithm parameters

///ARM 1
double a1PreviousError = 0;
double a1currentTime = 0;
double a1PreviousTime = 0;

double a1IntegralSum = 0;

double tests = 18;

double a1timeStamp = 0;
double a1timeStamp2 = 0;

// PID gains
double arm1Kp = 0.1;//0.1;// * (1/pressure);
double arm1Ki = 0.002;//0.001;// * (1/pressure);
double arm1Kd = 0.000001;//0.0001;// * (1/pressure);

///ARM 2
double a2PreviousError = 0;
double a2currentTime = 0;
double a2PreviousTime = 0;

double a2IntegralSum = 0;

double a2timeStamp = 0;
double a2timeStamp2 = 0;

// PID gains
double arm2Kp = 0.03;//0.1;// * (1/pressure);
double arm2Ki = 0.0001;//0.000001;//0.001;// * (1/pressure);
double arm2Kd =0.000001;// 0.00000001;//0.0001;// * (1/pressure);

int onTime;//no longer using?
int offTime;

//stepper motor vcc is yellow, gnd is white
//blue wired from module goes to gnd arduino
const int stepPin = 6; //white
const int dirPin = 5; //green
const int enPin = 4;//red

const int encoder1 = A0;
const int encoder2 = A1;

int lastTriggerTime = 0;

int stepCount = 0;
int encoder1Val = 0;
int encoder2Val = 0;

Servo myServo;

int step = 1;

//SOLENOID PIN NUMBERS, (*the yellow wires are the output pin-1s and the blue are output pin-2s*)
int a1mag1 = 10;
int a1mag2 = 12;

int a2mag1 = 9;
int a2mag2 = 13;

int clawMag = 11;

int lastTime = 0;

//Pressure of my system
int pressure = 125;//psi

double maxPower = 1.0;

//OUT/IN and ON & OFF OUTPUTS FOR THE SOLENOIDS, (3 TOTAL)
//ARM1
void solenoidArm1outOn(){
  digitalWrite(a1mag1, HIGH);//a posittive vs. negative 2-pin config is unnessicary in this case
}
void solenoidArm1outOff(){
  digitalWrite(a1mag1, LOW);
}
void solenoidArm1inOn(){
  digitalWrite(a1mag2, HIGH);
}
void solenoidArm1inOff(){
  digitalWrite(a1mag2, LOW);
}
//ARM2
void solenoidArm2outOn(){
  digitalWrite(a2mag1, HIGH);
}
void solenoidArm2outOff(){
  digitalWrite(a2mag1, LOW);
}
void solenoidArm2inOn(){
  digitalWrite(a2mag2, HIGH);
}
void solenoidArm2inOff(){
  digitalWrite(a2mag2, LOW);
}
//CLAW
void solenoidClawOn(){
  digitalWrite(clawMag, HIGH);
}
void solenoidClawOff(){
  digitalWrite(clawMag, LOW);
}

double pwmPIDarm1(double currentAngle, double targetAngle, double time, double systemPressure){//the greater the pressure, I think the lower the pid gains because the system would be more reactive

  //in this case, tha pwm amplitude will essentially be 0 or 5v (then the relay.. so final output amplitude has a height of 12v max)
  //the period of might need to be increased becuase of   ...slow system response time in the solenoid :(
 
  //error
  double error = targetAngle - currentAngle;

  //the time difference offset

  a1currentTime = time;
  
  double deltaTime = a1currentTime - a1PreviousTime;
  if (deltaTime == 0) deltaTime = 0.001; //technically necessary in some other algorithms like this as not to divid by 0, although, this will never actually happen bc arduino takes time to loop

  //Before, it would still pulse when the error was really high, so we were jsut never getting to target from the start
  double proportionalTerm = 0;
  //proportional
  if(abs(error) > 10){
    proportionalTerm = error;
  }else{
    proportionalTerm = arm1Kp * error;
  }
  //integral, sum of error

  a1IntegralSum += error * deltaTime;
  double integralTerm = arm1Ki * a1IntegralSum;

  // error as derivitive
  double derivativeTerm = arm1Kd * (error - a1PreviousError) / deltaTime;

  //out
  double output = proportionalTerm + integralTerm + derivativeTerm;

  //This stores current error and time for next iteration
  a1PreviousError = error;
  a1PreviousTime = a1currentTime;

  //the above output is the rate at which θ needs to change, but, must  put in rate of the pneum piston with respect to current (dθ/dt)
    //(although, perhaps this is slightly irrelevant during the first phase of motion...after all we want to get there fast as possible)

  //normalize outputs
  if (output > maxPower){
    output = maxPower;
  }
  if (output < -maxPower){
    output = -maxPower;
  }

  //output = map(output, -1, 1, 0, 1); //previously I used this 0-1 system for the double acting solenoids

  return output;//the pulse for the solenoid of arm1
  //"power" is pwm in this case, with a always going to be an instintaneously binary output

}

double pwmPIDarm2(double currentAngle, double targetAngle, double time, double systemPressure){//same thing as above for the most part
  //error
  double error = targetAngle - currentAngle;

  //the time difference offset

  a2currentTime = time;
  
  double deltaTime = a2currentTime - a2PreviousTime;
  if (deltaTime == 0) deltaTime = 0.001; 
 
  double proportionalTerm = 0;
  //proportional
  if(abs(error) > 15){//10 degree intolerance
    proportionalTerm = error;
  }else{
    proportionalTerm = arm2Kp * error;
  }
  

  //integral, sum of error

  a2IntegralSum += error * deltaTime;
  double integralTerm = arm2Ki * a2IntegralSum;

  // error as derivitive
  double derivativeTerm = arm2Kd * (error - a2PreviousError) / deltaTime;

  //out
  double output = proportionalTerm + integralTerm + derivativeTerm;

  // Store current error and time for next iteration
  a2PreviousError = error;
  a2PreviousTime = a2currentTime;

  //normal outputs
  if (output > maxPower){
    output = maxPower;
  }
  if (output < -maxPower){
    output = -maxPower;
  }

  return output;//the pulse for the solenoid of arm2

}


//PIN SETUP and others
void setup() {


  //for stepper motor
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);

  digitalWrite(enPin,LOW);
  Serial.begin(9600);

  //for servo
  myServo.attach(7);//white wire

  //for solenoids
  pinMode(a1mag1, OUTPUT);
  pinMode(a1mag2, OUTPUT);
  pinMode(a2mag1, OUTPUT);
  pinMode(a2mag2, OUTPUT);
  
  pinMode(clawMag, OUTPUT);

  step = 1;

  delay(5000);
 lastTime = millis();
}

void loop() {

  Action target = doTheStuffs[targetNum];
  if(millis() - lastTime > waitTime && targetNum < 11){//the number of actions - 1
    lastTime = millis();
    targetNum += 1;
  }

  stepTargetPos = target.getStepTarget();
  targetA1 = target.geta1Angle();
  targetA2 = target.geta2Angle();
  targetA3 = target.geta3Angle();
  waitTime = target.getmilliTime();
  clawOn = target.getclawState();

  //Serial.println(angle2);

  if(stepPos > stepTargetPos){
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin,LOW);
  
    digitalWrite(stepPin, LOW);
    delayMicroseconds(60);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);

    stepPos-= 1;

  }else if(stepPos < stepTargetPos){
    digitalWrite(enPin,LOW);
    digitalWrite(dirPin,HIGH);
  
    digitalWrite(stepPin, LOW);
    delayMicroseconds(60);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);

    stepPos += 1;

  }else{
    digitalWrite(enPin,HIGH);
  }

  currentEncoderTime = millis();

  if(currentEncoderTime-lastEncoderTime > 20){//20millis between readings

    encoder1Val = analogRead(encoder1);//gets encoder val
    encoder2Val = analogRead(encoder2);

    lastEncoderTime = currentEncoderTime;

  	//the pt calculations should also go in here

    Coordinate pt2 = Coordinate((arm1Len * cos(angle1) + pt1.getX()), (arm1Len * sin(angle1) + pt1.getY()));//equation to calculate pt2

    //Pt3 is the servo pivot point, pivot for arm3
    //origin relative angle2:
    orgRelAng2 = angle2 + angle1;
    Coordinate pt3 = Coordinate((arm2Len * cos(orgRelAng2) + pt2.getX()), (arm2Len * sin(orgRelAng2) + pt2.getY()));

    //origin relative angle3:
    angle3 = servoTargetAngle;//because there is no way to measure live with this system
    orgRelAng3 = orgRelAng2 + angle3;
    Coordinate pt4 = Coordinate((arm3Len * cos(orgRelAng3) + pt3.getX()), (arm3Len * sin(orgRelAng3) + pt3.getY()));

    //Serial.println(pt4.getX(), pt4.getY());

  }

  angle1 = map(encoder1Val, 350, 800, 155, 65);
  angle2 = map(encoder2Val, 200, 650, 140, 45);
//800 (in), 350 (out)

  //int servoAngle = map(angle, 0, 290, 0, 270);

  myServo.write(targetA3);

  //FOR THE SOLENOIDS:
  double a1percentOn = pwmPIDarm1(angle1, targetA1, millis(), 60);

  double a2percentOn = pwmPIDarm2(angle2, targetA2, millis(), 60);

  //ARM 1 OUTPUT CONTROL
  if(a1percentOn > 0){
    
    if(millis() < a1timeStamp + (70 * abs(a1percentOn))){
      solenoidArm1outOn();
  
      if(abs(a1percentOn) > 0.1){//adds this protection so it doesnt keep triggering when its in the equqlibrium state
        solenoidArm1inOff();
      }

      a1timeStamp2 = millis();
    }else{
      if(a1percentOn < 1){
        solenoidArm1inOn();
      }
      //solenoidArm1inOn();
      if(millis() > a1timeStamp2 + 70 * (1-abs(a1percentOn))){
        a1timeStamp = millis();
      }
    }

  }
  if(a1percentOn < 0){
    
    if(millis() < a1timeStamp + (70 * abs(a1percentOn))){
      solenoidArm1inOn();

      if(abs(a1percentOn) > 0.1){//tuned
        solenoidArm1outOff();
      }

      a1timeStamp2 = millis();
    }else{
      if(abs(a1percentOn) < 1){
        solenoidArm1outOn();
      }
      //solenoidArm1outOn();

      if(millis() > a1timeStamp2 + 70 * (1-abs(a1percentOn))){
        a1timeStamp = millis();
      }
    }

  }
  //Serial.println(a2percentOn);
  //ARM 2 OUTPUT CONTROL
  if(a2percentOn < 0){
    
    if(millis() < a2timeStamp + (50 * abs(a2percentOn))){
      solenoidArm2outOn();
      if(abs(a2percentOn) > 0.1){//adds this protection so it doesnt keep triggering when its in the equqlibrium state
        solenoidArm2inOff();
      }
      a2timeStamp2 = millis();
    }else{
      if(abs(a2percentOn) < 1){
        solenoidArm2inOn();
      }
      if(millis() > a2timeStamp2 + 50 * (1-abs(a2percentOn))){//lets say the percentOn is 30%, then it will be off for 60 millis seconds
        a2timeStamp = millis();
      }
    }

  }else if(a2percentOn > 0){
    
    if(millis() < a2timeStamp + (50 * abs(a2percentOn))){
      
      solenoidArm2inOn();
      
      if(abs(a2percentOn) > 0.1){//adds this protection so it doesnt keep triggering when its in the equqlibrium state
        solenoidArm2outOff();
      }

      a2timeStamp2 = millis();
    }else{
      if(abs(a2percentOn) < 1){
        solenoidArm2outOn();
      }
      if(millis() > a2timeStamp2 + 50 * (1-abs(a2percentOn))){
        a2timeStamp = millis();
      }
    }

  }else{//the holding power is roughghly alwasy the same
    solenoidArm2outOff();//not helpful actually
    solenoidArm2inOff();
    /*if(millis() < a2timeStamp + (100 * abs(1))){
      
      solenoidArm2outOn();
      solenoidArm2inOn();

      a2timeStamp2 = millis();
    }else{
      
      //solenoidArm2outOff();
      //solenoidArm2inOn();
      
      if(millis() > a2timeStamp2 + 100 * (1-abs(0))){
        a2timeStamp = millis();
      }
    }*/
    
  }




  if (clawOn){
    solenoidClawOn();
  }else{
    solenoidClawOff();
  }


//the servo can use external power supply, but the gnd has to be connected to arduino
 
//END OF LOOP
}

//PWM is from 0 - 255. In this case, 0 = -1, 255 = 1, and the rest is an in between
//PID will make a target "speed," and map everything into PWM
//may need to use related rates to calculate smoothly
    //yes actually, I think the equation is 2l(dl/dt) = -2h(d3)(-sinθ)(dθ/dt), where l is length of main pneum actuator pivot from theta pivot, and h is the height from that




