#define MaxPWM 100
#define td
#define h 0.001


float a,b;
float kp= ,
		ki= ,
		kd= ;
float error;

float kt = 10*ki;

int8_t uP = 0, uI = 0, uD = 0;

int8_t ut, u;
int8_t derror;

int8_t dD;


void Control_Temp(float presetTemp){
	duty = 0xFFFF; /*sets RPM to 0*/
	Fan_SetDuty(duty);
	
	a = td/(td+h); //Konstanten ausrechnen
	b = kd/(td+h); //Konstanten ausrechnen
	
	
	static float currentTemperature;
	
	SI7021_ReadTemperatureHold(&currentTemperature); //read Temperature
	
	error = presetTemp-currentTemperature;
	
	uP = kp * error;

	uI = uI + (ki*h*error) - kt*h*(ut-u);
	
	uD = a*dD + b*(error - derror);
	
	ut = uP + uI + uD;
	
	if(ut > MaxPWM){
		ut = MaxPWM;
	}
	
	if(ut < 0){
		ut = 0;
	}
	
	heaterSetPWM(u);//richtige Funktion wählen für PWM-Heizung
	
	dD = D;
	derror = error;
	

}