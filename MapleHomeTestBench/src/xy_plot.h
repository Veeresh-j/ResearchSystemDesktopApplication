/*
 * xy_plot1.h
 *
 *  Created on: 13-Aug-2020
 *      Author: ubuntu
 */

#ifndef XY_PLOT_H_
#define XY_PLOT_H_


#include <slope/slope.h>
#include "utility.h"


#define DISP_NUM_SAMPLES 			200
#define MAX_SAMPLES         		6000
#define MAX_YVAL_MV         		200


typedef struct{
	GtkWidget *views;
	SlopeItem *item[26];
	double time_axis[MAX_SAMPLES];
	double e1[MAX_SAMPLES];
	double e2[MAX_SAMPLES];
	double e3[MAX_SAMPLES];
	double e4[MAX_SAMPLES];
	double e5[MAX_SAMPLES];
	double e6[MAX_SAMPLES];
	double e7[MAX_SAMPLES];
	double e8[MAX_SAMPLES];
	double e9[MAX_SAMPLES];
	double e10[MAX_SAMPLES];
	double e11[MAX_SAMPLES];
	double e12[MAX_SAMPLES];
	double e13[MAX_SAMPLES];
	double e14[MAX_SAMPLES];
	double e15[MAX_SAMPLES];
	double e16[MAX_SAMPLES];
	double e17[MAX_SAMPLES];
	double e18[MAX_SAMPLES];
	double e19[MAX_SAMPLES];
	double e20[MAX_SAMPLES];
	double e21[MAX_SAMPLES];
	double e22[MAX_SAMPLES];
	double e23[MAX_SAMPLES];
	double e24[MAX_SAMPLES];
	double eavg[MAX_SAMPLES];
	double marker_val[MAX_SAMPLES];
}STRUCT_1;


typedef struct{
	double x_max_range;
	double y_max_range;
}XY_RANGE;

//to change the background and fg color

XY_RANGE xy_axis_range;
GtkWidget *window_main;
GtkWidget *primary_box,*notify_box,*box_for_graph,*button_box1;
GtkWidget *start_button,*pause_button,*save_button, *close_button,*elec_switcher, *reconnect_button;
GtkWidget *switch_eleclbl,*label2,*label3;
GtkWidget *x_adj_button, *y_adj_button;
GdkColor color;
STRUCT_1 cb;

typedef struct
{
	int measurement_type;
	int active_time;
	int rest_time;
	int contraction_count;
	int measure_duration;
	int emg_sensitivity;
	int emg_timespan;
	int activetime_count;
	int rest_timecount;
	int measurementTotTimeCount;
}MEASUREMENT_PARAM;

MEASUREMENT_PARAM meausrement_settings;


//static int windowLengthSamplesG=0;
 void set_yaxis_range(void);

void chart_init();
//void timer_callback_2(RMS_INFO *rms_cb);
gboolean timer_callback(STRUCT_1 *cb_func);


#endif /* XY_PLOT_H_ */
