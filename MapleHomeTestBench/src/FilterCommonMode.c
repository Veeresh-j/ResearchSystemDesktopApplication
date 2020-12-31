/*
 * FilterCommonMode.c
 *
 *  Created on: 08-Jul-2020
 *      Author: ubuntu
 */
#include "FilterCommonMode.h"

//Common mode filter.
//When channel is not active, then the common of that channel will not be used in the calculations.
//The inactive channel will be subtracted with the common signal that is used on the other channels.
void FilterCommonMode(float* cin, float* cout, uint_32 channels);


void FilterCommonMode(float* cin, float* cout, uint_32 channels)
{
	double total = 0;
	double buffer[24];
	uint_32 mask = 1;
	uint_32 divide = 0;

	for (int i=0; i < MAXCHANNELS; i++)
	{
		buffer[i] = (double)cin[i]; //store the input

		if(channels & mask) //only count common mode for active channels
		{
			total += buffer[i];
			divide++;
		}
		mask <<= 1;
	}
	if(divide == 0) //prevent divide by zero
	{
		total = 0;
		divide = 1;
	}
	double avg = total/divide;

	for (int i=0; i < MAXCHANNELS; i++) //generate the output
	{
		cout[i] = (float)(buffer[i] - avg); 	//this could overflow, but will then be clipped. This will only happen with strange signals,
	}								//for example 23 channels high positive common mode and 1 channel high negative signal.

}


void Averaging(float input[], float output[], uint_32 channels,int samplecount)
{
	float* pInput = &input[0];
	float* pOutput = &output[0];


	for(int z=0; z < samplecount; z += MAXCHANNELS)
	{
		FilterCommonMode(pInput,pOutput,0xFFFFFF);
		pInput += MAXCHANNELS;
		pOutput += MAXCHANNELS;

	}
}



