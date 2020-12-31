#include <stdbool.h>
#include "utility.h"
//#define RTLSIMULATION //only define RTLSIMULATION when running a C/RTL Cosimulation, undefine for C simulation or C synthesis
#define TEST_60HZ //test 60Hz RMS (RMS window is 102 samples)
//#define TEST_50HZ //test 50Hz RMS (RMS window is 100 samples)
//50Hz and 60Hz must run seperately because each run requires its own port depth of the #pragma HLS INTERFACE.

#ifdef TEST_50HZ
#undef TEST_60HZ
#endif

//#include <ap_fixed.h>

//typedef ap_fixed<32,24,AP_RND_ZERO,AP_SAT> fixed_24_8;
//typedef ap_fixed<72,56,AP_RND_ZERO,AP_SAT> fixed_56_16;
typedef signed long uint_64;


//! calculate RMS signal of input.
//! this filter has a rate conversion from 1000Hz to 10Hz (window size = 100) for 50Hz filter configuration
//! this filter has a rate conversion from 1020Hz to 10Hz (window size = 102) for 60Hz filter configuration
void calculate_rms(double* cin, double* cout, bool freq_60);
