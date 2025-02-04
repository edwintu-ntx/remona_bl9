/**********************************************************************
 *
 *     pid.h		Propotional-Intergation-Differentation controls
 *
 *
 *
 **********************************************************************/

//etu added
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

 /*pid param struct */
 typedef struct
 {
     float target_val;     //Target Value
     float actual_val;     //Real Value
     float err;            //current error
     float err_next;       //next error
     float err_last;       //last error
     float Kp, Ki, Kd;     //Kp, Ki, Kd define
 }Pid_Remona, *pPid_Remona;

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

typedef struct pidcontainer
{
	int kp;			// Proportional Constant
	int ki;			// Intergration Constant
	int kd;			// Defferentation Constant
	int kg;			// Output Gain Constant

	int err;		// Current Error
	long int intg;	// Intergration holder
	int diff;		// Differantion holder
	long int lasterr;	// Error of last time, to calulater Differeantation
}_pidcontainer;

/* * *  Prototypes */
void init_pid( struct pidcontainer *s, int kp, int ki, int kd, int kg );
int pid( struct pidcontainer *s, int error );

void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);
float RemonaSystem_Update(float inp);

void PID_param_init( pPid_Remona pid);
float PID_Realize(float temp_val, pPid_Remona pid);

#endif
