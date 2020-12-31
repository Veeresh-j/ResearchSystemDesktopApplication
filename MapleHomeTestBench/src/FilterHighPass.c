/*
 * FilterHighPass.c
 *
 *  Created on: 07-Jul-2020
 *      Author: ubuntu
 */

#include "FilterHighPass.h"

typedef struct
{
	float x1;
	float y1;
}dataset;



float FilterHighPass_run(dataset* data, float x0, double b1, double b2, double a2);
void FilterHighPass(float* cin, float* cout,FREQUENCY freq,bool reset_toggle);


float FilterHighPass_run(dataset* data, float x0, double b1, double b2, double a2)
{
	// a1*y(n) = b1*x(n)+b2*x(n-1)-a2*y(n-1), a1 = 1
	// split calculations over several lines!!!
	// otherwise, you get MaxInt (see testRun2). Has probably to do with types of intermediate values
	double result = (b1 * x0); 		//b1 * x(n)
	result += (b2 * data->x1);		//b2 * x(n-1)
	result -= (a2 * data->y1);		//a2 * y(n-1)
	// store for next iteration
	data->y1 = result;
	data->x1 = x0;
	// result
	return result;
}


void FilterHighPass(float* cin, float* cout,FREQUENCY freq,bool reset_toggle)
{
	// 50Hz, 900sps
	double coefs_50[9] ={0.9695312529087462, -0.9695312529087462, -0.9390625058174924,
							0.9662575430688309, -0.9662575430688309, -0.9325150861376618,
							0.9630050766175764, -0.9630050766175764, -0.9260101532351529  };

	static dataset filter[3][MAXCHANNELS];
	static double reg_coef_1;
	static double reg_coef_2;
	static double reg_coef_3;
	static double reg_coef_4;
	static double reg_coef_5;
	static double reg_coef_6;
	static double reg_coef_7;
	static double reg_coef_8;
	static double reg_coef_9;
	float after9;
	float after10;
	static bool reset = true;
	static bool reset_toggle_prev = false;

	// configure filter
	if (reset || (reset_toggle_prev != reset_toggle))
	{
		//configure filter
		if(freq == HERTZ50)
		{
			reg_coef_1 = coefs_50[0];
			reg_coef_2 = coefs_50[1];
			reg_coef_3 = coefs_50[2];
			reg_coef_4 = coefs_50[3];
			reg_coef_5 = coefs_50[4];
			reg_coef_6 = coefs_50[5];
			reg_coef_7 = coefs_50[6];
			reg_coef_8 = coefs_50[7];
			reg_coef_9 = coefs_50[8];
		}

		for (int i = 0; i < MAXCHANNELS; i++)
		{
			filter[0][i].x1 = 0;
			filter[0][i].y1 = 0;
			filter[1][i].x1 = 0;
			filter[1][i].y1 = 0;
			filter[2][i].x1 = 0;
			filter[2][i].y1 = 0;
		}
		reset = false;
		reset_toggle_prev = reset_toggle;
	}

	// run filters
	//! High pass 1st order butterworth filter that is executed 3 times for
	//! frequencies 9Hz, 10Hz, 11Hz

	for (int i = 0; i < MAXCHANNELS; i++)
	{
		after9  = FilterHighPass_run(&filter[0][i], cin[i], reg_coef_1, reg_coef_2, reg_coef_3);
		after10 = FilterHighPass_run(&filter[1][i], after9, reg_coef_4, reg_coef_5, reg_coef_6);
		cout[i] = FilterHighPass_run(&filter[2][i], after10, reg_coef_7, reg_coef_8, reg_coef_9);
	}

}

void HighPassFilter(float input[], float output[],int samplecount)
{
	float* pInput = &input[0];
	float* pOutput = &output[0];


	for(int z=0; z <= samplecount; z += MAXCHANNELS)
	{
		FilterHighPass(pInput, pOutput,HERTZ50, true);
		pInput += MAXCHANNELS;
		pOutput += MAXCHANNELS;

	}

}







