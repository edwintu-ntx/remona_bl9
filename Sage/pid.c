/**********************************************************************
 *
 *     pid.c		Propotional-Intergation-Differentation controls
 *
 *
 *
 **********************************************************************/

#include "pid.h"


#define LASTERRORSMOOTHING (4) /* How many samples that the last error is based on */
/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f

#define PID_TAU 0.02f

#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

//etu added
extern float setpoint;
extern float measurement;
//etu

//PIDController pid_Temp;
/* Initialise PID controller */
PIDController pid_Temp = { PID_KP, PID_KI, PID_KD,
            	PID_TAU,
            	PID_LIM_MIN, PID_LIM_MAX,
				PID_LIM_MIN_INT, PID_LIM_MAX_INT,
            	SAMPLE_TIME_S };

//void PID_param_init(struct pid_Remona pid);
//float PID_Temp_Realize(float temp_Val);

/**
 * @brief  PID Param Init
 * @note   None
 * @retval None
 */
 void PID_param_init(pPid_Remona pid)
 {
     /* Init Parameters */
//     printf("PID_init begin \n");
     pid->target_val= 0.0;
     pid->actual_val=0.0;
     pid->err = 0.0;
     pid->err_last = 0.0;
     pid->err_next = 0.0;
     //	pid.Kp = 0.21;
     // pid.Ki = 0.070;
     // pid.Kd = 0.32;
     pid->Kp = 0.21;
     pid->Ki = 0.80;
     pid->Kd = 0.01;
//     printf("PID_init end \n");
 }


/**
 * @brief  PID Realize
 * @param  val Target 
 * @note   None
 * @retval Output After PID Calculation 
 */
float PID_Realize(float temp_val, pPid_Remona pid)
{
 /*Calculate the error between target and real value*/
     pid->err=pid->target_val-temp_val;
     /*PID Calculation Impelentation*/
     float increment_val = pid->Kp*(pid->err - pid->err_next) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
     /*Accumlated*/
     pid->actual_val += increment_val;
     /*passing error*/
     pid->err_last = pid->err_next;
     pid->err_next = pid->err;
     /*return real value*/
     return pid->actual_val;
}

#if 0
void pid_Compute(void)
{

//	float t;
    /* Initialise PID controller */
//    PIDController pid_Temp = { PID_KP, PID_KI, PID_KD,
//            	PID_TAU,
//            	PID_LIM_MIN, PID_LIM_MAX,
//				PID_LIM_MIN_INT, PID_LIM_MAX_INT,
//            	SAMPLE_TIME_S };

    PIDController_Init(&pid_Temp);

    /* Simulate response using test system */
//    float setpoint = 1.0f;

    printf("Time (s)\tSystem Output\tControllerOutput\r\n");
    for (t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {

        /* Get measurement from system */
        float measurement = RemonaSystem_Update(pid_Temp.out);

        /* Compute new control signal */
        PIDController_Update(&pid_Temp, setpoint, measurement);

        printf("%f\t%f\t%f\r\n", (double)t, (double)measurement, (double)pid_Temp.out);

    }

}
#endif

/* ---------------------------------------------------
 *
 *  init_pid( pidstruct *s, int kp, int ki, int kd, int kg)
 *
 *  Initilize the pidstruct
 *
 *     kp, ki, and kd are in percentage,
 *     so a kp of 100 is equal to Proportial constant of 1
 *
 *     kg is the overal gain of the PID, to make up for small constants.
 *     it is also in percentage, so...
 *     kg =1000 is a gain of 10.00
 *
 * --------------------------------------------------*/
#if 0
void init_pid( struct pidcontainer *s, int kp, int ki, int kd, int kg )
{
	s->kp = kp;
	s->ki = ki;
	s->kd = kd;
	s->kg = kg;

	s->lasterr = 0L;
	s->err     = 0;

	s->intg   =0L;
	s->diff   =0;

}
#endif

/* ---------------------------------------------------
 *
 *  pid( struct pidcontainer *s, int error )
 *
 *  returns the output of a PID, based on the latest error
 *
 * --------------------------------------------------*/
#if 0
int pid( struct pidcontainer *s, int error )
{
	long int newout;

	s->err  = error;
	s->intg += s->err;
	s->diff  =  s->err - ((int) s->lasterr/LASTERRORSMOOTHING);

	/* Clip Intgral from getting out of bounds! */
    if( s->intg > 32767L ) // Is intg at positive saturation?
		s->intg = 32767L;		// YES, clip it to MAXINT

    if( s->intg < -32767L ) // Is intg at negative saturation?
		s->intg = -32767L;		// YES, clip it to Negitive MAXINT

	/* Calutate new output value of PID */
	newout  = ( (long) s->kp * (long) s->err ) / 100L ;
	newout += ( (long) s->ki * (long) s->intg ) / 100L ;
	newout += ( (long) s->kd * (long) s->diff ) / 100L ;
	newout  = ( (long) s->kg * newout ) / 100L;

	/* Clip newout from getting out of bounds, and keep intg from winding up! */
    if( newout > 32767L ) 	// Is newout at positive saturation?
    {
		newout = 32767L;	// YES, clip it to MAXINT
		s->intg -= s->err;	// And undo intergation (anti-windup)
    }

//    if( newout < 0 ) 		// Is newout at negative saturation?
//    {
//		newout = 0;			// YES, clip it to 0 percent
//		s->intg -= s->err;	// And undo intergation (anti-windup)
//    }

    if( newout < -32767L )          // Is newout at negative saturation?
    {
        newout = -32767L;           // YES, clip it to Negitive MAXINT
        s->intg -= s->err;          // And undo intergation (anti-windup)
    }

	/* Save last error for differential (smoothed version) */
	s->lasterr += (long) s->err - s->lasterr/LASTERRORSMOOTHING;

	return ( ((int) newout) / 327 );			// return back as a percentage

}
#endif

void PIDController_Init(PIDController *pid) {

	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* Error signal
	*/
    float error = setpoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pid->Kp * error;


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}

float RemonaSystem_Update(float inp) {

    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

    return output;
}