#include "max6675.h"
#include <PID_v1.h>
//#include <string.h>

#define RelayPin 6  //Relay + pin
#define RelayGnd 7
#define LEDPin 4  //Status led pin
#define LEDGnd 3
#define SENSOR_SAMPLING_TIME 1000 //read tc every second
#define thermoVCC 10
#define thermoSO 11
#define thermoCS 12
#define thermoCSK 13
#define PIDWindowSize 1000

int preheatTemp = 180;

//profile stuff
byte currentStep = 1;
byte profileSteps = 8;
double rampRates[11] 	= {0,	1,	1,	1,	1,	1,	1,	1,	1};
int dwellTimers[11] 	= {0,	20,	30,	33,	20,	20,	30,	200,	200};
int temperatures[11] 	= {0,	150,	180,	190,	200,	210,	220,	230,	235};

long previousMillis; //these are for counters
double counter;

//these are the different states of the sketch. We call different ones depending on conditions
typedef enum REFLOW_STATE
{
  REFLOW_STATE_RAMP,
  REFLOW_STATE_STEP,
  REFLOW_STATE_DWELL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_ERROR,
  REFLOW_STATE_IDLE,
  REFLOW_STATE_TEST,
  REFLOW_STATE_HON,
  REFLOW_STATE_HOFF,
  REFLOW_STATE_STOP,
  REFLOW_STATE_CONNECTING,
  REFLOW_STATE_JOB_START
} 
reflowState_t;

typedef enum REFLOW_STATUS //this is simply to check if reflow should be running or no
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} 
reflowStatus_t;

reflowStatus_t reflowStatus;
reflowState_t reflowState;


//PID stuff
double Setpoint = 0;
double Input = 0;
double Output = 0; 

double RampP = 300;
double RampI = 0.025;
double RampD = 20;

double DwellP = 300;
double DwellI = 0.05;
double DwellD = 250;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,RampP,RampI,RampD, DIRECT);
unsigned long windowStartTime;

//Thermocouple

//TC read timer
unsigned long nextRead;
MAX6675 thermocouple(thermoCSK, thermoCS, thermoSO);

double currTemp;

//#P8#120_20_0.25#
//convert single line string to profile
void loadProfile(char *line)
{
    
}

char rcvBuff[100];


void setup()
{
	//start serial port
	Serial.begin(9600);
	  
	//setup relay pins
	pinMode(RelayPin, OUTPUT);
	pinMode(RelayGnd, OUTPUT);
	digitalWrite(RelayGnd, LOW);
	digitalWrite(RelayPin, LOW);
  
	//setup led pins
	pinMode(LEDPin, OUTPUT);
	pinMode(LEDGnd, OUTPUT);
	digitalWrite(LEDGnd, LOW);
	digitalWrite(LEDPin, HIGH);   
  
	//Setup thermocouple pins
	pinMode(thermoVCC, OUTPUT); 
	digitalWrite(thermoVCC, HIGH);
	
	// wait for MAX chips to stabilize  
	delay(1000); 
  
	digitalWrite(LEDPin, LOW);
	
	// Initialize  top thermocouple reading variable
	nextRead = millis();
	
	//Total time sketch has been running
	windowStartTime = millis();
	
	//myPID1 = top heater PID loop
	myPID.SetOutputLimits(0, PIDWindowSize);
	myPID.SetMode(AUTOMATIC);
  
	reflowState = REFLOW_STATE_IDLE;
}



void loop()
{
	unsigned long currentMillis = millis();
	if (nextRead <= currentMillis)
	{
		nextRead += SENSOR_SAMPLING_TIME;
		Input = thermocouple.readCelsius();
	}

  
	switch (reflowState)
	{
	case REFLOW_STATE_CONNECTING:  
		Serial.print("#ACK");    
		reflowState = REFLOW_STATE_IDLE; 
	break;
    
    
	case REFLOW_STATE_IDLE:
		Serial.println("IDLE"); 
      /*if (Serial.available > 0)
      {      
        Serial.readBytes(rcvBuff);      
        if (compareTo(rcvBuff, "#START") == 0)
        {
          reflowStatus = REFLOW_STATUS_ON;
          reflowState = REFLOW_STATE_STEP;
        }
        else if (compareTo(rcvBuff, "#TEST") == 0)
        {
          reflowState = REFLOW_STATE_TEST;
        }
      }
      else
      {*/   
          reflowStatus = REFLOW_STATUS_ON;
          reflowState = REFLOW_STATE_STEP;
          Serial.println("Step 1");
      //}
      
    break;    
    
	case REFLOW_STATE_RAMP:
	
		if(currentMillis - previousMillis > 1000 / rampRates[currentStep]) 
		{
			previousMillis = currentMillis;
			counter++;
			Setpoint = temperatures[currentStep-1] + counter;
		} 

		if (Setpoint >= temperatures[currentStep]) 
		{
			reflowState = REFLOW_STATE_STEP;
			Serial.print("STEP "); Serial.println(currentStep);
		}
  
	break;
  
	case REFLOW_STATE_STEP:
		Setpoint = temperatures[currentStep];   
		if (Input >= temperatures[currentStep])
		{
			myPID.SetTunings(DwellP, DwellI, DwellD);
			counter = 0;
			reflowState = REFLOW_STATE_DWELL;
			Serial.print("DWELL step "); Serial.println(currentStep);
		}  
  
	break;
  
	case REFLOW_STATE_DWELL:
  
		if(currentMillis - previousMillis > 1000) 
		{
			previousMillis = currentMillis;
			counter++;
		} 
		if (counter == dwellTimers[currentStep]) 
		{
			counter = 0;
			if (profileSteps == currentStep) 
			{
				reflowState = REFLOW_STATE_COMPLETE;
			}
			else
			{	
				myPID.SetTunings(RampP, RampI, RampD);
				reflowState = REFLOW_STATE_RAMP;
				currentStep++;
				Serial.print("RAMP step "); Serial.println(currentStep);
			}
		}
	break; 


	case REFLOW_STATE_COMPLETE:
		Serial.write("ZAVRSENO");
    
		reflowStatus = REFLOW_STATUS_OFF;
		reflowState = REFLOW_STATE_STOP;
    
    break;
		
    case REFLOW_STATE_STOP:
		digitalWrite(LEDPin, HIGH);
		delay(500);
		digitalWrite(LEDPin, LOW);
		delay(500);
		
    break;
	}
	
	
  
	if (reflowStatus == REFLOW_STATUS_ON)
	{
		myPID.Compute(); 

		if (millis() - windowStartTime>PIDWindowSize)
		{ //time to shift the Relay Window			
			windowStartTime += PIDWindowSize;
			Serial.print("In: "); Serial.print(Input);
			Serial.print(" Set: "); Serial.print(Setpoint);
			Serial.print(" Out: "); Serial.println(Output);
		}
		if (Output > millis() - windowStartTime) 
		{
			digitalWrite(RelayPin,HIGH); 
			digitalWrite(LEDPin, HIGH);
		}
		else 
		{
			digitalWrite(RelayPin,LOW);
			digitalWrite(LEDPin, LOW);    
		}
	}
	else 
	{
		digitalWrite(RelayPin, LOW);
		digitalWrite(LEDPin, LOW); 
	}
}
