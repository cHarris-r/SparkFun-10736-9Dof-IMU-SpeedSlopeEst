/*******************************************************************
** Types and defines for DSP Functions
** This file is used to aid in the common
** DSP functions which we may use to scrub 
** the incomming sensor data
********************************************************************/



/*******************************************************************
** DEFINES
********************************************************************/

/* Define the number of taps in filter */
#define NTAPS 5

/* FIR Filter coeffs
** All FIR filters have a cutoff of 0.1*nyq 
** Equation:
**	y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[N]*x[n-N]
*/

#define FIR_LPF_9 {0.014408,0.043863,0.120212,0.202534,0.237966,0.202534,0.120212,0.043863,0.014408}
#define FIR_LPF_5 {0.033833,0.240127,0.452079,0.240127,0.033833}
#define FIR_LPF_3 {0.067990,0.864020,0.067990}

#define FIR_HPF_9 {-0.006092,-0.018545,-0.050824,-0.085629,0.905484,-0.085629,-0.050824,-0.018545,-0.006092}
#define FIR_HPF_5 {-0.007550,-0.053584,0.907931,-0.053584,-0.007550}
#define FIR_HPF_3 {-0.008593,0.982814,-0.008593}

#if NTAPS==3
	#define FIR_LPF FIR_LPF_3
	#define FIR_HPF FIR_HPF_3
#elif NTAPS==5
	#define FIR_LPF FIR_LPF_5
	#define FIR_HPF FIR_HPF_5
#elif NTAPS==9
	#define FIR_LPF FIR_LPF_9
	#define FIR_HPF FIR_HPF_9
#endif

/* IIR Fitler Coeffs 
** All IIR Filters are Butterworth by design
** with a cutoff of 0.1
** Equation:
**	y[n] = (1/a[0]) * ( b[0]*x[n] + ... + b[P]*x[n-P] - a[1]*y[n-1] - ... - a[Q]*y[n-Q] )
*/

#define IIR_LPF_3a {1.000000,-1.561018,0.641352}
#define IIR_LPF_3b {0.020083,0.040167,0.020083}
#define IIR_LPF_5a {1.000000,-3.180639,3.861194,-2.112155,0.438265}
#define IIR_LPF_5b {0.000417,0.001666,0.002500,0.001666,0.000417}
#define IIR_LPF_9a {1.000000,-6.390365,18.000338,-29.171099,29.731375,-19.505632,8.040996,-1.903669,0.198100}
#define IIR_LPF_9b {0.000000,0.000001,0.000005,0.000010,0.000012,0.000010,0.000005,0.000001,0.000000}

#define IIR_HPF_3a {1.000000,-1.561018,0.641352}
#define IIR_HPF_3b {0.800592,-1.601185,0.800592}
#define IIR_HPF_5a {1.000000,-3.180639,3.861194,-2.112155,0.438265}
#define IIR_HPF_5b {0.662016,-2.648063,3.972095,-2.648063,0.662016}
#define IIR_HPF_9a {1.000000,-6.390365,18.000338,-29.171099,29.731375,-19.505632,8.040996,-1.903669,0.198100}
#define IIR_HPF_9b {0.445084,-3.560674,12.462360,-24.924719,31.155899,-24.924719,12.462360,-3.560674,0.445084}

#if NTAPS==3
	#define IIR_LPF_a IIR_LPF_3a
	#define IIR_LPF_b IIR_LPF_3b
	#define IIR_HPF_a IIR_HPF_3a
	#define IIR_HPF_b IIR_HPF_3b
#elif NTAPS==5
	#define IIR_LPF_a IIR_LPF_5a
	#define IIR_LPF_b IIR_LPF_5b
	#define IIR_HPF_a IIR_HPF_5a
	#define IIR_HPF_b IIR_HPF_5b
#elif NTAPS==9
	#define IIR_LPF_a IIR_LPF_9a
	#define IIR_LPF_b IIR_LPF_9b
	#define IIR_HPF_a IIR_HPF_9a
	#define IIR_HPF_b IIR_HPF_9b
#endif

/*******************************************************************
** TYPEDEFS
********************************************************************/

typedef struct
{
	float IIR_coeffs_La[NTAPS] = IIR_LPF_a;
	float IIR_coeffs_Lb[NTAPS] = IIR_LPF_b;
	float IIR_coeffs_Ha[NTAPS] = IIR_HPF_a;
	float IIR_coeffs_Hb[NTAPS] = IIR_HPF_b;
	
	float FIR_coeffs_L[NTAPS] = FIR_LPF;
	float FIR_coeffs_H[NTAPS] = FIR_HPF;
	
	float accel_mem[3][NTAPS];
	float gyro_mem[3][NTAPS];
} DSP_COMMON_TYPE;
	
	
	
	
	
	