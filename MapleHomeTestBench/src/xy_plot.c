

#include <stdint.h>
#include <stdbool.h>
#include <slope/slope.h>
#include "xy_plot.h"
#include "utility.h"


#define Y_SCALE_MIN				0
#define Y_SCALE_MAX				200

SlopeScale *  scale1 , *scale2, *scale3, *scale4, *scale5, *scale6, *scale7, *scale8, *scale9, *scale10, *scale11, *scale12,*scaleavg;
SlopeItem *   series1, *series2,*series3,*series4,*series5, *series6, *series7,*series8,*series9,*series10, *series11,*series12,*seriesavg;
GtkWidget *view;
SlopeFigure *figure;

int fillPointerLow;
int fillPointerHigh;
int displayPointer;


void marker_init(void);


void chart_init()
{

			fillPointerLow = 0;
			displayPointer = 1;
			fillPointerHigh = xy_axis_range.x_max_range;

			view = slope_view_new();

			figure = slope_figure_new();

			gtk_box_pack_start(GTK_BOX(box_for_graph), view, TRUE, TRUE, 0);

			gtk_box_pack_start(GTK_BOX(box_for_graph), button_box1, FALSE,TRUE , 0);

			gtk_box_pack_start(GTK_BOX(button_box1), reconnect_button, FALSE, FALSE,10);
			gtk_box_pack_start(GTK_BOX(button_box1), switch_eleclbl, FALSE, FALSE,10);
			gtk_box_pack_start(GTK_BOX(button_box1), elec_switcher, FALSE, FALSE,10);
			gtk_box_pack_start(GTK_BOX(button_box1), start_button, FALSE, TRUE,10);
			gtk_box_pack_start(GTK_BOX(button_box1), pause_button, FALSE, TRUE, 10);
			gtk_box_pack_start(GTK_BOX(button_box1), save_button, FALSE, TRUE, 10);
			gtk_box_pack_start(GTK_BOX(button_box1), close_button, FALSE, TRUE, 10);


			gtk_box_pack_start(GTK_BOX(button_box1), label2, TRUE, TRUE, 2);
			gtk_box_pack_start(GTK_BOX(button_box1), x_adj_button, FALSE, TRUE, 2);

			gtk_box_pack_start(GTK_BOX(button_box1), label3, FALSE, TRUE, 2);
			gtk_box_pack_start(GTK_BOX(button_box1), y_adj_button, FALSE, TRUE, 2);


			slope_view_set_figure(SLOPE_VIEW(view), figure);


		  series1 =slope_xyseries_new();
		  series2 =slope_xyseries_new();
		  series3 =slope_xyseries_new();
		  series4 =slope_xyseries_new();
		  series5 =slope_xyseries_new();
		  series6 =slope_xyseries_new();
		  series7 =slope_xyseries_new();
		  series8 =slope_xyseries_new();
		  series9 =slope_xyseries_new();
		  series10 =slope_xyseries_new();
		  series11 =slope_xyseries_new();
		  series12 =slope_xyseries_new();
		  seriesavg =slope_xyseries_new();

/*		  slope_item_set_name(series1, "e1");
		  slope_item_set_name(series2, "e2");
		  slope_item_set_name(series3, "e3");
		  slope_item_set_name(series4, "e4");
		  slope_item_set_name(series5, "e5");
		  slope_item_set_name(series6, "e6");
		  slope_item_set_name(series7, "e7");
		  slope_item_set_name(series8, "e8");
		  slope_item_set_name(series9, "e9");
		  slope_item_set_name(series10, "e10");
		  slope_item_set_name(series11, "e11");
		  slope_item_set_name(series12, "e12");*/
		  slope_item_set_name(seriesavg, "avg");


		  scale1 = slope_xyscale_new();
		 // slope_scale_set_background_color(scale1, SLOPE_BLACK);
		  slope_xyscale_set_axis(SLOPE_XYSCALE(scale1), SLOPE_XYSCALE_FRAME_LINE);
		  slope_xyseries_set_style(SLOPE_XYSERIES(series1), "r-");
		  slope_figure_add_scale(SLOPE_FIGURE(figure), scale1);
		  slope_scale_set_layout_rect(scale1, 0,0.0,1.0,0.130);
		  slope_scale_add_item(scale1, series1);

			scale2 = slope_xyscale_new();
			//slope_scale_set_layout_rect(scale2, 0,0.092, 1, 0.22);
			//slope_scale_set_background_color(scale2, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale2), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale2);
			slope_xyseries_set_style(SLOPE_XYSERIES(series2), "r-");
			slope_scale_set_layout_rect(scale2, 0,0.07, 1.0, 0.130);
			//slope_scale_set_layout_rect(scale2, 0,0.0835, 1, 0.1);
			slope_scale_add_item(scale2, series2);

			scale3 = slope_xyscale_new();
		  	//slope_scale_set_background_color(scale3, SLOPE_BLACK);
		    slope_xyscale_set_axis(SLOPE_XYSCALE(scale3), SLOPE_XYSCALE_FRAME_LINE);
		  	//slope_chart_add_scale(SLOPE_CHART(chart), scale3);
		  	slope_figure_add_scale(SLOPE_FIGURE(figure), scale3);
			slope_xyseries_set_style(SLOPE_XYSERIES(series3), "r-");
			slope_scale_set_layout_rect(scale3, 0,0.14, 1, 0.130);
			slope_scale_add_item(scale3, series3);

		  	scale4 = slope_xyscale_new();
		  	//slope_scale_set_background_color(scale4, SLOPE_BLACK);
		  	slope_xyscale_set_axis(SLOPE_XYSCALE(scale4), SLOPE_XYSCALE_FRAME_LINE);
		  	slope_figure_add_scale(SLOPE_FIGURE(figure), scale4);
		  	//slope_chart_add_scale(SLOPE_CHART(chart), scale4);
			slope_xyseries_set_style(SLOPE_XYSERIES(series4), "r-");
		 	slope_scale_set_layout_rect(scale4, 0,0.21, 1, 0.130);
			slope_scale_add_item(scale4, series4);

		  	scale5 = slope_xyscale_new();
			//slope_scale_set_background_color(scale5, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale5), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale5);
			slope_xyseries_set_style(SLOPE_XYSERIES(series5), "r-");
			slope_scale_set_layout_rect(scale5, 0,0.28, 1, 0.130);
			slope_scale_add_item(scale5, series5);

			scale6 = slope_xyscale_new();
			//slope_scale_set_background_color(scale6, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale6), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale6);
			slope_xyseries_set_style(SLOPE_XYSERIES(series6), "r-");
			slope_scale_set_layout_rect(scale6, 0,0.35, 1, 0.130);
			slope_scale_add_item(scale6, series6);

			scale7 = slope_xyscale_new();
			//slope_scale_set_background_color(scale7, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale7), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale7);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale7);
			slope_xyseries_set_style(SLOPE_XYSERIES(series7), "r-");
			slope_scale_set_layout_rect(scale7, 0,0.42, 1, 0.130);
			slope_scale_add_item(scale7, series7);

			scale8 = slope_xyscale_new();
			//slope_scale_set_background_color(scale8, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale8), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale8);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale8);
			slope_xyseries_set_style(SLOPE_XYSERIES(series8), "r-");
			slope_scale_set_layout_rect(scale8, 0,0.49, 1, 0.130);
			slope_scale_add_item(scale8, series8);

			scale9 = slope_xyscale_new();
			//slope_scale_set_background_color(scale9, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale9), SLOPE_XYSCALE_FRAME_LINE);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale9);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale9);
			slope_xyseries_set_style(SLOPE_XYSERIES(series9), "r-");
			slope_scale_set_layout_rect(scale9, 0,0.56, 1, 0.130);
			slope_scale_add_item(scale9, series9);

			scale10 = slope_xyscale_new();
			//slope_scale_set_background_color(scale10, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale10), SLOPE_XYSCALE_FRAME_LINE);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale10);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale10);
			slope_xyseries_set_style(SLOPE_XYSERIES(series10), "r-");
			slope_scale_set_layout_rect(scale10, 0,0.63, 1, 0.130);
			slope_scale_add_item(scale10, series10);


			scale11 = slope_xyscale_new();
			//slope_scale_set_background_color(scale11, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale11), SLOPE_XYSCALE_FRAME_LINE);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale11);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale11);
			slope_xyseries_set_style(SLOPE_XYSERIES(series11), "r-");
			slope_scale_set_layout_rect(scale11, 0,0.70, 1, 0.130);
			slope_scale_add_item(scale11, series11);

			scale12 = slope_xyscale_new();
			//slope_scale_set_background_color(scale12, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scale12), SLOPE_XYSCALE_FRAME_LINE);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale12);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scale12);
			slope_xyseries_set_style(SLOPE_XYSERIES(series12), "r-");
			slope_scale_set_layout_rect(scale12, 0,0.77, 1, 0.130);
			slope_scale_add_item(scale12, series12);

			scaleavg = slope_xyscale_new();
			//slope_scale_set_background_color(scaleavg, SLOPE_BLACK);
			slope_xyscale_set_axis(SLOPE_XYSCALE(scaleavg), SLOPE_XYSCALE_FRAME_AXIS_GRID);
			//slope_chart_add_scale(SLOPE_CHART(chart), scale12);
			slope_figure_add_scale(SLOPE_FIGURE(figure), scaleavg);
			slope_xyseries_set_style(SLOPE_XYSERIES(seriesavg), "b-");
			slope_scale_set_layout_rect(scaleavg, 0,0.86, 1, 0.150);
			slope_scale_add_item(scaleavg, seriesavg);

			cb.views = view;
			cb.item[0] = series1;
			cb.item[1] = series2;
			cb.item[2] = series3;
			cb.item[3] = series4;
			cb.item[4] = series5;
			cb.item[5] = series6;
			cb.item[6] = series7;
			cb.item[7] = series8;
			cb.item[8] = series9;
			cb.item[9] = series10;
			cb.item[10] = series11;
			cb.item[11] = series12;
			cb.item[12] = seriesavg;

			for (int k = 0; k < xy_axis_range.x_max_range*2; ++k)
			{
				cb.time_axis[k] = k;
			}
			if(meausrement_settings.measurement_type > 1)
			{
				marker_init();
			}
			set_yaxis_range();
}

gboolean timer_callback(STRUCT_1 *cb_func)
{
	bool retval = true;
	uint16_t disp_samples = xy_axis_range.x_max_range;//DISP_NUM_SAMPLES;


	//xRange = 10 * gtk_spin_button_get_value(GTK_SPIN_BUTTON(x_adj_button));
	//yRange = gtk_spin_button_get_value(GTK_SPIN_BUTTON(y_adj_button));


	displayPointer = fillPointerLow+1;
	if(displayPointer >= xy_axis_range.x_max_range) displayPointer=0;

	if(gtk_switch_get_active(GTK_SWITCH(elec_switcher)) == FALSE)
	{
		slope_item_set_name(series1, "e1");
		slope_item_set_name(series2, "e2");
		slope_item_set_name(series3, "e3");
		slope_item_set_name(series4, "e4");
		slope_item_set_name(series5, "e5");
		slope_item_set_name(series6, "e6");
		slope_item_set_name(series7, "e7");
		slope_item_set_name(series8, "e8");
		slope_item_set_name(series9, "e9");
		slope_item_set_name(series10, "e10");
		slope_item_set_name(series11, "e11");
		slope_item_set_name(series12, "e12");

		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[0]), (cb_func->time_axis+displayPointer),(cb_func->e1+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[1]), (cb_func->time_axis+displayPointer),(cb_func->e2+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[2]), (cb_func->time_axis+displayPointer),(cb_func->e3+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[3]), (cb_func->time_axis+displayPointer),(cb_func->e4+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[4]), (cb_func->time_axis+displayPointer),(cb_func->e5+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[5]), (cb_func->time_axis+displayPointer),(cb_func->e6+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[6]), (cb_func->time_axis+displayPointer),(cb_func->e7+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[7]), (cb_func->time_axis+displayPointer),(cb_func->e8+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[8]), (cb_func->time_axis+displayPointer),(cb_func->e9+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[9]), (cb_func->time_axis+displayPointer),(cb_func->e10+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[10]), (cb_func->time_axis+displayPointer),(cb_func->e11+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[11]), (cb_func->time_axis+displayPointer),(cb_func->e12+displayPointer), disp_samples);

	}
	else
	{
		slope_item_set_name(series1, "e13");
		slope_item_set_name(series2, "e14");
		slope_item_set_name(series3, "e15");
		slope_item_set_name(series4, "e16");
		slope_item_set_name(series5, "e17");
		slope_item_set_name(series6, "e18");
		slope_item_set_name(series7, "e19");
		slope_item_set_name(series8, "e20");
		slope_item_set_name(series9, "e21");
		slope_item_set_name(series10, "e22");
		slope_item_set_name(series11, "e23");
		slope_item_set_name(series12, "e24");

		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[0]), (cb_func->time_axis+displayPointer),(cb_func->e13+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[1]), (cb_func->time_axis+displayPointer),(cb_func->e14+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[2]), (cb_func->time_axis+displayPointer),(cb_func->e15+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[3]), (cb_func->time_axis+displayPointer),(cb_func->e16+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[4]), (cb_func->time_axis+displayPointer),(cb_func->e17+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[5]), (cb_func->time_axis+displayPointer),(cb_func->e18+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[6]), (cb_func->time_axis+displayPointer),(cb_func->e19+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[7]), (cb_func->time_axis+displayPointer),(cb_func->e20+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[8]), (cb_func->time_axis+displayPointer),(cb_func->e21+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[9]), (cb_func->time_axis+displayPointer),(cb_func->e22+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[10]), (cb_func->time_axis+displayPointer),(cb_func->e23+displayPointer), disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[11]), (cb_func->time_axis+displayPointer),(cb_func->e24+displayPointer), disp_samples);

	}

	slope_xyseries_update_data(SLOPE_XYSERIES(cb_func->item[12]), (cb_func->time_axis+displayPointer),(cb_func->eavg+displayPointer), disp_samples);

	if(meausrement_settings.measurement_type > 1)
	{
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[13]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[14]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[15]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[16]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[17]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[18]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[19]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[20]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[21]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[22]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[23]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[24]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[25]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
		slope_xyseries_update_data(SLOPE_XYSERIES(cb.item[26]), (cb_func->time_axis+displayPointer),(cb.marker_val+displayPointer),disp_samples);
	}

	set_yaxis_range();
	slope_view_redraw(SLOPE_VIEW(cb_func->views));


   return retval;
}


/*bool firstTime = true;
static void idle_callback()
{
	if(firstTime ==true)
	{
		g_timeout_add (100,(GSourceFunc)timer_callback,(gpointer)&cb);
		firstTime=false;
	}

}*/

 void set_yaxis_range(void)
{
	if(scale1!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale1),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale2!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale2),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale3!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale3),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale4!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale4),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale5!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale5),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale6!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale6),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale7!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale7),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale8!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale8),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale9!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale9),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale10!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale10),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale11!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale11),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scale12!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scale12),Y_SCALE_MIN,xy_axis_range.y_max_range);
	if(scaleavg!=NULL) slope_xyscale_set_y_range(SLOPE_XYSCALE(scaleavg),Y_SCALE_MIN,xy_axis_range.y_max_range);

}

 void marker_init(void)
 {
  	cb.item[13] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[13]), "la");

  	cb.item[14] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[14]), "la");

  	cb.item[15] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[15]), "la");

  	cb.item[16] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[16]), "la");

  	cb.item[17] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[17]), "la");

  	cb.item[18] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[18]), "la");

  	cb.item[19] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[19]), "la");

  	cb.item[20] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[20]), "la");

  	cb.item[21] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[21]), "la");

  	cb.item[22] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[22]), "la");

  	cb.item[23] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[23]), "la");

  	cb.item[24] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[24]), "la");

  	cb.item[25] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[25]), "la");

  	cb.item[26] =slope_xyseries_new();
  	slope_xyseries_set_style(SLOPE_XYSERIES(cb.item[26]), "la");

  	slope_scale_add_item(scale1, cb.item[13]);
  	slope_scale_add_item(scale2, cb.item[14]);
  	slope_scale_add_item(scale3, cb.item[15]);
  	slope_scale_add_item(scale4, cb.item[16]);
  	slope_scale_add_item(scale5, cb.item[17]);
  	slope_scale_add_item(scale6, cb.item[18]);
  	slope_scale_add_item(scale7, cb.item[19]);
  	slope_scale_add_item(scale8, cb.item[20]);
  	slope_scale_add_item(scale9, cb.item[21]);
  	slope_scale_add_item(scale10,cb.item[22]);
  	slope_scale_add_item(scale11,cb.item[23]);
  	slope_scale_add_item(scale12,cb.item[24]);
  	slope_scale_add_item(scaleavg,cb.item[25]);
  }
