/*
 * FilterComb.c
 *
 *  Created on: 08-Jul-2020
 *      Author: ubuntu
 */

#include "FilterComb.h"

#define MAXDELAY 		20


typedef struct
{
	double BodyLDelay21_DSTATE[MAXDELAY];
	double BodyLDelay41_DSTATE[MAXDELAY];
	double BodyRDelay21_DSTATE[MAXDELAY];
	double BodyRDelay41_DSTATE[MAXDELAY];
}comb_dataset;

double combfilt_coeffs_50[5] ={0.82263893079677, -1.64527786159354, 0.82263893079677,
						-1.61356955860502, 0.676986164582056  };


double FilterComb_run(comb_dataset* data, double input, double b_01, double b_21, double b_41, double a_21, double a_41, bool freq_60);
void FilterComb(float* cin, float* cout,FREQUENCY freq, bool reset_toggle);


//Comb filter with 2 selectable delays: 50Hz and 60Hz
double FilterComb_run(comb_dataset* data, double input, double b_01, double b_21, double b_41, double a_21, double a_41, bool freq_60)
{
	  /* Delay: '<S1>/BodyLDelay21' */
	double rtb_BodyLDelay21 = data->BodyLDelay21_DSTATE[0];

	  /* Delay: '<S1>/BodyRDelay21' */
	double rtb_BodyRDelay21 = data->BodyRDelay21_DSTATE[0];


	long double rtb_BodyRSum41;
	rtb_BodyRSum41 =  b_01 * input;
	rtb_BodyRSum41 += b_21 * data->BodyLDelay21_DSTATE[0];
	rtb_BodyRSum41 += b_41 * data->BodyLDelay41_DSTATE[0];
	rtb_BodyRSum41 -= a_21 * data->BodyRDelay21_DSTATE[0];
	rtb_BodyRSum41 -= a_41 * data->BodyRDelay41_DSTATE[0];

	double output = (double) rtb_BodyRSum41;

	unsigned int delay;
	if(freq_60)
	{
		//delay = 17-1; //1020/60
		delay = 15-1; //900/60
	}
	else
	{
		//delay = 20-1; //1000/50
		delay = 18-1; //900/50
	}


	for (unsigned int idxDelay = 0; idxDelay < delay; idxDelay++)
	{
		/* Update for Delay: '<S1>/BodyLDelay21' */
		data->BodyLDelay21_DSTATE[idxDelay] = data->BodyLDelay21_DSTATE[idxDelay + 1];

		/* Update for Delay: '<S1>/BodyLDelay41' */
		data->BodyLDelay41_DSTATE[idxDelay] = data->BodyLDelay41_DSTATE[idxDelay + 1];

		/* Update for Delay: '<S1>/BodyRDelay21' */
		data->BodyRDelay21_DSTATE[idxDelay] = data->BodyRDelay21_DSTATE[idxDelay + 1];

		/* Update for Delay: '<S1>/BodyRDelay41' */
		data->BodyRDelay41_DSTATE[idxDelay] = data->BodyRDelay41_DSTATE[idxDelay + 1];
	}

	/* Update for Delay: '<S1>/BodyLDelay21' incorporates:
	 *  Update for Inport: '<Root>/Input'
	 */
	data->BodyLDelay21_DSTATE[delay] = input;

	/* Update for Delay: '<S1>/BodyLDelay41' */
	data->BodyLDelay41_DSTATE[delay] = rtb_BodyLDelay21;

	/* Update for Delay: '<S1>/BodyRDelay21' */
	data->BodyRDelay21_DSTATE[delay] = output;

	/* Update for Delay: '<S1>/BodyRDelay41' */
	data->BodyRDelay41_DSTATE[delay] = rtb_BodyRDelay21;

	return output;
}

void FilterComb(float* cin, float* cout,FREQUENCY freq, bool reset_toggle)
{
	static double reg_b_01;
	static double reg_b_21;
	static double reg_b_41;
	static double reg_a_21;
	static double reg_a_41;
	static bool reg_freq_60;

	static comb_dataset filterdata[MAXCHANNELS];
	//static comb_dataset filterdata;

	static bool reset = true;

	static bool reset_toggle_prev = false;

	// configure filter
	if (reset || (reset_toggle_prev != reset_toggle))
	{
		if(freq == HERTZ50)
		{
			reg_b_01 = combfilt_coeffs_50[0];
			reg_b_21 = combfilt_coeffs_50[1];
			reg_b_41 = combfilt_coeffs_50[2];
			reg_a_21 = combfilt_coeffs_50[3];
			reg_a_41 = combfilt_coeffs_50[4];
			reg_freq_60 = false;
		}
		for (int i1=0; i1 < MAXCHANNELS; i1++)
		{
			for(int i2=0; i2 < MAXDELAY; i2++)
			{
				filterdata[i1].BodyLDelay21_DSTATE[i2] = 0;
				filterdata[i1].BodyLDelay41_DSTATE[i2] = 0;
				filterdata[i1].BodyRDelay21_DSTATE[i2] = 0;
				filterdata[i1].BodyRDelay41_DSTATE[i2] = 0;
			}
		}
		reset = false;
		reset_toggle_prev = reset_toggle;
	}

	// run filters
	for (int i = 0; i < MAXCHANNELS; i++)
	{
		cout[i] = (float)FilterComb_run(&filterdata[i], (double)cin[i], reg_b_01, reg_b_21, reg_b_41, reg_a_21, reg_a_41, reg_freq_60);
	}
}

void CombFilter(float input[], float output[],int samplecount)
{
	float* pInput = &input[0];
	float* pOutput = &output[0];


	for(int z = 0; z < samplecount; z += MAXCHANNELS)
	{
		FilterComb(pInput, pOutput,HERTZ50, true);
		pInput += MAXCHANNELS;
		pOutput += MAXCHANNELS;

	}

}



