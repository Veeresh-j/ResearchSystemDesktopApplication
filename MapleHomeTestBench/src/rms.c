#include "rms.h"

uint_64 isqrt1(uint_64 n, int max_iterations);

static const int max_iterations = 50;    // max iterations for isqrt. calculated that this is  the minimum for the given window size.

/* Newton's method for square root: x(i+1) = 1/2 * (x(i) + n/x(i))
 */
uint_64 isqrt1(uint_64 n, int max_iterations)
{
	uint_64 x0 = 1;
	uint_64 xi = 0;
	for (int i = 0; i < max_iterations; i++)
	{
		xi = ((n / x0) + x0) >> 1;
		x0 = xi;
	}
	return xi;
}

void calculate_rms(double* cin, double* cout, bool freq_60)
{
	// limit number of multipliers
	long double running_squares[MAXCHANNELS * 40];
	int window_size;

	if(freq_60)
		window_size = 102;
	else
		window_size = 100;

	int index = 0;
	for (int window_position=0; window_position < window_size; window_position++)
	{
		for (int channel_id=0; channel_id < (MAXCHANNELS*40); channel_id++)
		{
			// calculate running squares
			double input_sample = cin[index];
			running_squares[channel_id] += input_sample * input_sample;
			index++;
		}
	}
	// calculate rms
	for (int channel_id=0; channel_id<(MAXCHANNELS*40); channel_id++)
	{
		long double temp = running_squares[channel_id] / window_size;
	//	cout[channel_id] = isqrt1(temp, max_iterations);
		cout[channel_id]  = sqrt(temp);
	}
}
