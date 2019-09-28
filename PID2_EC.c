#include<stdio.h>
#include<time.h>
#include<stdbool.h>
#include<unistd.h>
#include<string.h>
#include<math.h>

clock_t start, end;

// working variables 
unsigned long lastTime ;
double Input , Output , Setpoint ;
double ITerm , lastInput ;
double kp , ki , kd ;
int SampleTime = 1000; //1sec
double outMin , outMax ;
bool inAuto = false ;

#define SAMPLE_NUMBER 15 //24x6 sample

unsigned long hour = 3600000;
double list_EC [SAMPLE_NUMBER +1] ; 
double list_Output[SAMPLE_NUMBER +1] ;
int a_index =0;
double EC_target_start;
double EC_target_end;
unsigned long previoustime =0;

#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1


int controllerDirection = DIRECT;

int StringToNumber(char *s_Input);
void Compute();
void SetTunings ( double Kp, double Ki , double Kd);
void SetSampleTime ( int NewSampleTime );
void SetOutputLimits (double Min , double Max);
void Initialize ();
void SetMode (int Mode);
void SetControllerDirection ( int Direction );
int Add_to_array( double Input,double Output);
void Pump_solution_AB( double Output);
void Pump_water( double Output);


int main(int argc, char* argv[])
{
	SetTunings(20,5,0.1);
	SetSampleTime (1000) ;
	SetControllerDirection(DIRECT) ;
	SetMode (AUTOMATIC) ;	
	if(argc == 3)
	{
		double setValue1 = (double)StringToNumber(argv[1]);
		if(setValue1 <0) 
		{
			printf("ERROR - Setpoint Value\n");
		}
		double setValue2 = (double)StringToNumber(argv[2]);
		if(setValue2 <0) 
		{
			printf("ERROR - Input Value\n");
		}

		Setpoint = setValue1;
		Input = setValue2;
		EC_target_end = ((Setpoint-50)<0)?(0):(Setpoint-50);
		EC_target_start = Setpoint+50;

		printf("SETUP Setpoint - Input : %.0f - %.0f\n--------------------------------------------\n",Setpoint,Input);
		outMin = -5555;
		outMax = 5555;
		start = clock();
		do
		{
			Compute();

		}while(1);
	}
	else
	{
		printf("NOTE* : Command Line Arguments [ Setpoint Value ]\n");
		return -1;
	}
}

int StringToNumber(char *s_Input)
{
	int i,value =0;
	for(i=0;i<strlen(s_Input);i++)
	{
		if(s_Input[i]<'0' || s_Input[i]>'9') return -1;
				// ex. 123 = 1*10^2 + 2*10^1 + 3*10^0
		value+=(s_Input[i]-48)*(pow(10,(strlen(s_Input)- 1 -i)));  
	}
	return value;
}

int Add_to_array( double Input,double Output)
{
	if(a_index < SAMPLE_NUMBER)
	{
		list_EC[a_index]=Input;
		list_Output[a_index] = Output;
		a_index++;
	}
	else
	{
		for(int i =0; i < SAMPLE_NUMBER-1; i++)
		{
			list_EC[i] = list_EC[i+1];
			list_Output[i] = list_Output[i+1];
		}
		list_EC[SAMPLE_NUMBER-1] = Input;
		list_Output[SAMPLE_NUMBER-1] = Output;
	}
	return 0;
}

//=========================================================================================================================================
void Compute()
{
	if ( !inAuto ) return ;
	unsigned long now = (clock()-start)/CLOCKS_PER_SEC*1000;
	int timeChange = (now - lastTime);
	if (timeChange>=SampleTime )
	{
		// Compute all the working error variables 
		double error = Setpoint - Input ;
		ITerm += ( ki*error ) ;
		if ( ITerm > outMax ) ITerm= outMax ;
		else if ( ITerm < outMin ) ITerm= outMin ;
		double dInput = ( Input - lastInput ) ;
		// Compute PID Output
		Output = kp*error + ITerm - kd*dInput ;
		if ( Output > outMax ) Output = outMax ;
		else if ( Output < outMin ) Output = outMin ;
		// Remember some variables for next time 
		lastInput = Input ;
		lastTime = now ;
		
		Add_to_array(Input,Output);

		printf ( "\nEC	: " ) ;
		for ( int i = 0 ; i < SAMPLE_NUMBER ; i++) {
			printf("%.0f	",list_EC[i]);
		}
		printf ( "\nOUTPUT	: ");
		for ( int i = 0 ; i < SAMPLE_NUMBER ; i++) {
			printf("%.0f	",list_Output[i]);
		}
		printf("\n");
		// printf("\nInput - Output : %f - %f  ", Input, Output);
		if ( Input <EC_target_start ) {
			Pump_solution_AB(Output);
		}
		else if( Input > EC_target_end ) {
			Pump_water(Output);
		}
		Input = Input + (Output/100.0f);
		if(now - previoustime >=1000*120)
		{
			Input -= 2222;
			previoustime = now;
		}
	}
}

void SetTunings ( double Kp, double Ki , double Kd)
{
	if (Kp<0 || Ki <0 || Kd<0) return ;

	double SampleTimeInSec = ((double)SampleTime)/1000;
	kp = Kp;
	ki = Ki*SampleTimeInSec ;
	kd = Kd/SampleTimeInSec ;

	if ( controllerDirection == REVERSE)
	{
		kp = (0 - kp ) ;
		ki = (0 - ki ) ;
		kd = (0 - kd ) ;
	}
}

void SetSampleTime ( int NewSampleTime )
{
	if (NewSampleTime > 0)
	{
		double ratio = (double)NewSampleTime/(double)SampleTime ;
		ki *= ratio ;
		kd /= ratio ;
		SampleTime = (unsigned long)NewSampleTime ;
	}
}

void SetOutputLimits (double Min , double Max)
{
	if (Min > Max) return ;
	outMin = Min ;
	outMax = Max;

	if ( Output > outMax ) Output = outMax ;
	else if ( Output < outMin ) Output = outMin ;

	if ( ITerm > outMax ) ITerm= outMax ;
	else if ( ITerm < outMin ) ITerm= outMin ;
}

void Initialize ()
{
	lastInput = Input ;
	ITerm = Output ;
	if ( ITerm > outMax ) ITerm= outMax ;
	else if ( ITerm < outMin ) ITerm= outMin ;
}

void SetMode (int Mode)
{
	bool newAuto = (Mode == AUTOMATIC) ;
	if ( newAuto == !inAuto )
	{ 
 	// we just went from manual to auto 
		Initialize();
	}
	inAuto = newAuto;
}
void SetControllerDirection ( int Direction )
{
	controllerDirection = Direction ;
}

void Pump_solution_AB( double Output)
{
	printf("==> Start pump Solution A&B \n");
}

void Pump_water( double Output)
{
	printf("==> Start pump water\n");
}