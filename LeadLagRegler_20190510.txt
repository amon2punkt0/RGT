/*
 * control.c
 *
 *  Created on: 10.12.2018
 *      Author: JumpStart
 */

#include "FRTOS1.h"
#include "Fan.h"
#include "Control.h"
#include "ADIN.h"
#include "Si7021.h"
#include "TU2.h"
#include "TU3.h"
#include "Heater.h"
#include "vl53l0x.h"
#include "Strobo_Init.h"
#include "Heater_Init.h"
//#include "GetDistance.h"

ControlData_t data;

#if CTRL_FAN_SPEED
/* PID CONTROL PARAMETERS ------------------------------------------------------*/
#define PID_K_P			(    1.0000000000 )	/* proportional					*/
#define PID_K_I			(     0.0500000000 )	/* integral 					*/
#define PID_K_D			(     0.0000000000 )	/* differential					*/
#define PID_K_FF_V		(     0.0040000000 )	/* feed-forward supply voltage	*/
/* PID SYSTEM LIMITATIONS ------------------------------------------------------*/
#define PID_CTRL_MIN	(           0x0000 )	/* max control value 			*/
#define PID_CTRL_MAX	(           0XFFFF )	/* min control value 			*/
/* PID TIMEBASE (dt) -----------------------------------------------------------*/
#define CTRL_TIMEBASE	(               20 )	/* dt = 20ms					*/
/* CONTROL FUNCTION	------------------------------------------------------------*/
void Control_FanSpeedManual(void);				/* control fan for manual input */
#endif /* CTRL_FAN_SPEED */

#if CTRL_BALL_HEIGHT
/* PID CONTROL PARAMETERS ------------------------------------------------------*/
#define PID_K_P			(    1.0000000000 )	/* proportional					*/ //1
#define PID_K_I			(     0.05000000000 )	/* integral 					*/ //0.05
#define PID_K_D			(     0.0000000000 )	/* differential					*/
/* PID SYSTEM LIMITATIONS ------------------------------------------------------*/
#define PID_CTRL_MIN	(           0x0000 )	/* max control value 			*/
#define PID_CTRL_MAX	(           0XFFFF )	/* min control value 			*/
/* CYLINDER SYSTEM ------------------------------------------------------*/
#define CylinderHeight	(           300 		)	/* cylinder height mm			*/
#define TargetBallDistanceFromSensor (  100 )	/* 		in mm		 			*/
/* PID TIMEBASE (dt) -----------------------------------------------------------*/
#define CTRL_TIMEBASE	(               20 )	/* dt = 20ms					*/
/* CONTROL FUNCTION	------------------------------------------------------------*/
void Control_Height(void);				/* control fan for manual input */
#endif /* CTRL_FAN_SPEED */

#if CTRL_TEMP
#define CTRL_TIMEBASE	(               20 )	/* dt = 20ms					*/
/* FAN SYSTEM LIMITATIONS ------------------------------------------------------*/
#define CTRL_MIN	(           0x0000 )	/* max control value 			*/
#define CTRL_MAX	(           0XFFFF )	/* min control value 			*/
/* HEATER LIMITATIONS ------------------------------------------------------*/
#define Heater_MAX	(           30 )		/*  Heater threshold 			*/
/* PID TIMEBASE (dt) -----------------------------------------------------------*/
#define CTRL_TIMEBASE	(               20 )	/* dt = 20ms					*/
/* CONTROL FUNCTION	------------------------------------------------------------*/
void Control_Temp(void);
#endif /* CTRL_TEMP */

#if CTRL_PLL
/* PID CONTROL PARAMETERS ------------------------------------------------------*/
#define PID_K_P			(  5000.0000000000 ) 	/* proportional 				*/
#define PID_K_I			(     0.1000000000 )	/* integral						*/
#define PID_K_D			(     0.0000000000 )	/* differential					*/
/* PID SYSTEM LIMITATIONS ------------------------------------------------------*/
#define	PID_CTRL_MIN	(            27000 )	/* min control value			*/
#define PID_CTRL_MAX	(		     35000 )	/* max control value			*/
#define CTRL_TIMEBASE	(               20 ) 	/* dt = 20ms					*/
void Control_PLL(void);
#endif /* CTRL_PLL */

static void ControlTask(void *pvParameters) {
	(void)pvParameters; /* not used */
	portTickType last_time_run;
	last_time_run = xTaskGetTickCount();
	for(;;) {
#if CTRL_FAN_SPEED
		Control_FanSpeedManual();
#endif /* CTRL_FAN_SPEED */
#if CTRL_BALL_HEIGHT
		Control_Height();
#endif /* CTRL_BALL_HEIGHT */
#if CTRL_TEMP
		Control_Temp();
#endif /* CTRL_TEMP */
#if CTRL_PLL
		Control_PLL();
#endif
		vTaskDelayUntil(&last_time_run, (CTRL_TIMEBASE/portTICK_PERIOD_MS));
	}
}

void Control_Init(void) {
	/* initialize data structure */
	data.fanDuty		= 0;
	data.fanRPM 		= 0;
	data.height			= 0;
	data.pressure 		= 0;
	data.temperature	= 0;
	data.time 			= 0;
	data.voltage		= 0;
	data.target			= 0;
	data.phase			= 0;

	/* create control task */
	if (FRTOS1_xTaskCreate(ControlTask, "Control", configMINIMAL_STACK_SIZE+100, NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) {
		for(;;){} /* error */
	}
}

void Control_SendData(ControlData_t *data) {
	unsigned char buf[64];

	/* send plot configuration value ----------------------------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"plot: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->plotConfig), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send time value (in ticks or steps, see CTRL_TIMEBASE) ---------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"time: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->time), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send height value (in millimeter) ------------------------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"height: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->height), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send temperature value (im millicelsius) -----------------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"temperature: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->temperature), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send pressure value (in millipascal) ---------------------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"pressure: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->pressure), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send fan duty value (absolute from 0x0 to 0xFFFF) --------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"fan_duty: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(0xFFFFUL - data->fanDuty), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send fan tacho value (in rpm) ----------------------------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"rpm: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num16uToStrFormatted(buf, sizeof(buf), (uint16_t)(data->fanRPM), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send system supply voltage value (in millivolts) ---------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"voltage: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->voltage), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send target value (relative from 0x0 to 0xFFFF) ----------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"target: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->target), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)" , ",  CLS1_GetStdio()->stdOut);

	/* send phase value (relative from 0x0 to 0xFFFF) ----------------------*/
#if (OUTPUT_MODE==OUTPUT_MODE_GUI) /* Send description only for GUI MODE */
	CLS1_SendStr((unsigned char*)"phase: ",  CLS1_GetStdio()->stdOut);
#endif
	buf[0] = '\0';
	UTIL1_Num32uToStrFormatted(buf, sizeof(buf), (uint32_t)(data->phase), ' ', 1);
	CLS1_SendStr(buf,  CLS1_GetStdio()->stdOut);
	CLS1_SendStr((unsigned char*)"\r\n ",  CLS1_GetStdio()->stdOut);
}

#if CTRL_FAN_SPEED
void Control_FanSpeedManual() {
	static uint8_t count = 0;
	static uint32_t time = 0;
	unsigned char buf[64];
	uint16_t currentSpeed;
	uint16_t targetSpeed;
	int32_t currentError;
	uint16_t voltage;
	static int32_t previousError = 0;
	double ctrl_p;
	static double ctrl_i;
	double ctrl_d;
	double ctrl_ff;
	double ctrl;
	uint16_t duty = 0;

	/* update control time */
	time++;

	/* measure target speed from manual input (potentiometer) */
	ADIN_Measure();
	voltage = ADIN_Convert_VPWR();
	targetSpeed = (ADIN_Convert_VMAN()*8)<<1;

	/* read current speed from tacho feedback */
	currentSpeed =Fan_GetRPM();



	// TODO: implement Ball Height Control loop

	/* calculate speed control error */

	/* calculate feed-forward voltage */

	/* saturation | limit */





	ctrl=0; // Delete this when the Control "ctrl" is written

	duty = 0xFFFF-(uint16_t)ctrl;
	Fan_SetDuty(duty);

	/* print current status every 200ms */
	if (count >= 10) {
		count = 0;

		data.plotConfig 	= PLOT_CONTROL_FAN_SPEED;
		data.time 			= time;
		data.fanDuty 		= duty;
		data.fanRPM 		= currentSpeed;
		data.target 		= targetSpeed;
		data.voltage 		= voltage;

		Control_SendData(&data);

	}
	else {
		count++;
	}
}
#endif /* CTRL_FAN_CONST_SPEED */

#if CTRL_BALL_HEIGHT
void Control_Height(void) {
	static uint8_t count = 0;
	static uint32_t time = 0;
	unsigned char buf[64];
	uint16_t currentSpeed;
	uint16_t targetSpeed;
	uint16_t distance;
	int32_t currentError;
	static int32_t previousError = 0;
	double ctrl_p;
	uint16_t voltage;
	static double ctrl_i;
	double ctrl_d;
	double ctrl_ff;
	double ctrl;
	uint32_t duty = 0;
	double delta = 0;
	int16_t sollDistanz = 150;

	/* update control time */
	time++;

	/* get distance from Sensor*/
	VL53L0X_MeasureSingle(&distance);


	/* read current speed from tacho feedback */
	currentSpeed =Fan_GetRPM();
	
	/* read voltage */
	voltage = ADIN_Convert_VPWR();



	// TODO: implement temperature Control loop

	/* calculate distance error in %  */

	/* calculate Controller */

	/* saturation | limit */

		currentError = sollDistanz - distance;//Fehler berechnen

		static double x1 = 0; //x1 und x2 initialisieren (nur beim ersten Mal)
		static double x2 = 0;

		static float h = 0.01; //Schrittweite (hier wird angenommen, dass Control_Height alle 10ms aufgerufen wird)

		//Euler-Approximation
		for(uint8_t j=0; j<9;j++){
			x1 = x1 + h * (1*x2);
			x2 = x2 + h * (-1*x1+-10.1*x2 + 1*currentError);
		}



		//if(time%10==0){//alle 100ms soll Ausgang ausgerechnet werden (hier wird angenommen, dass Control_Height alle 10ms aufgerufen wird)
			ctrl = -8.1*x2+1*currentError;//Ausgang y berechnen
			ctrl += 30500;//Offset für "Schweben" dazuzählen
		//}




		duty = 0xFFFF-(uint16_t)ctrl;
		Fan_SetDuty(duty);

	if (count >= 10) {
		count = 0;

		data.plotConfig 	= PLOT_CONTROL_BALL_HEIGHT;
		data.fanDuty		= duty;
		data.fanRPM 		= currentSpeed;
		data.height			= distance;
		data.time 			= time;

		data.voltage		= voltage;
		data.target			= targetSpeed;

		Control_SendData(&data);

	}
	else {
		count++;
	}
}
#endif /* CTRL_BALL_HEIGHT */

#if CTRL_TEMP
void Control_Temp(void) {
	static uint8_t count = 0;
	static uint32_t time = 0;
	static float currentTemperature;
	unsigned char buf[64];
	int32_t currentError;
	static int32_t previousError = 0;
	double ctrl_p;
	uint16_t voltage;
	static double ctrl_i;
	double ctrl_d;
	double ctrl_ff;
	double ctrl;
	uint32_t duty = 0;
	double delta = 0;
	uint16_t TargetTemperature;


	/* read current temperature */
	SI7021_ReadTemperatureHold(&currentTemperature);

	Heater_PutVal(HeaterTRUE);

	duty = 0xFFFF; /*sets RPM to 0*/
	Fan_SetDuty(duty);


	if (count >= 10) {
		count = 0;

		data.plotConfig 	= PLOT_CONTROL_TEMP;
		data.temperature	= currentTemperature;
		data.voltage		= voltage;
		data.target			= TargetTemperature;
		Control_SendData(&data);

	}
	else {
		count++;
	}

}
#endif /* CTRL_TEMP */

#if CTRL_PLL

void Control_PLL(void) {
	static uint32_t time = 0;
	static unsigned char count = 0;
	uint32_t deltaTime = 0;
	double timeDelay = 0;
	double phaseDelay = 0;
	uint16_t duty = 0;
	uint16_t offset = 31000;
	double ctrl;
	double ctrl_p;
	static double ctrl_i;
	double ctrl_d;


	uint16_t fanrpm;
	uint16_t voltage;

	/* update control time */
	time++;

	/* measure target speed from manual input (potentiometer) */
	ADIN_Measure();
	voltage = ADIN_Convert_VPWR();
	ADIN_Convert_VMAN();

	/* read current speed from tacho feedback */
	fanrpm = Fan_GetRPM();

	// TODO: implement phase-locked loop control

	ctrl=0; // Delete this when the Control "ctrl" is written

	/* set duty */
	duty = 0xFFFF - (uint16_t)ctrl;
	Fan_SetDuty(duty);

	/* print current status every 200ms */
	if (count >= 10) {
		count = 0;

		data.plotConfig 	= PLOT_CONTROL_PLL;
		data.time 			= time;
		data.height 		= 0;
		data.temperature 	= 0;
		data.pressure 		= 0;
		data.fanDuty 		= duty;
		data.fanRPM 		= fanrpm;
		data.voltage 		= voltage;
		data.target 		= PLLData.deltaTime;
		data.phase			= PLLData.phaseDelay;

		Control_SendData(&data);
	}
	else {
		count++;
	}

}
#endif /* CTRL_PLL */
