/*
 ============================================================================
 Name        : test.c
 Author      :
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */
// Linux headers for serial communication
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <string.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timeb.h>
#include "FilterHighPass.h"
#include "FilterComb.h"
#include "FilterCommonMode.h"
#include "utility.h"
#include <slope/slope.h>
#include "xy_plot.h"

#include <mysql.h>
#include <my_global.h>


#define BRAODCAST_IP_RANGE 			"192.168.0.255"
#define PORT	 					3000
#define CLIENTMACADR 				"98:FA:9B:07:67:73"
#define CLIENTMACADR_SIZE 			17
#define HEADERLEN 					12
#define PREFIX						0XFFFFFFFF
#define REQ_DISCOVERY				5
#define REQ_CONNECT 				24
#define REQ_INFOVERSION     		20
#define REQ_INFO_BATTERY     		21
#define REQ_MEAS_START    			50
#define REQ_MEAS_STOP    			51
#define REQ_SETFILTERFREQ    		925

#define RESP_DISCOVERY 				9051
#define RESP_MEAS_START				950
#define RESP_MEAS_STOP				951
#define RESP_MEAS_RAWDAT			952
#define RESP_BATINFO				921
#define RESP_MEAS_STOP				951
#define RESP_MEAS_RMS				953
#define RESP_MEAS_RMS_SUMMARY		954
#define RESP_MEAS_CON_START			955
#define RESP_MEAS_CON_STOP			956
#define RESP_MEAS_CON_SUMMARY		957

//#define ENABLE_CHANNELS			"111111111111111111111111"
#define ENABLE_CHANNELS				"111111111111111111111111"
#define INPUT_SAMPLERATE			1000
#define OUTPUT_SAMPLERATE_RMS			10
#define MAXTIME						240
#define MEAS_TYPE_RMS				1
#define MEAS_TYPE_RAW				0
#define REFERENCE_GROUP				1
#define CMP_REFERENCE_GROUP			1
#define OUTPUT_SAMPLERATE			10
#define INTIALIZATION_TIME			1
#define ACTIVE_TIME					3
#define REST_TIME					3
#define FILTER_FREQ					50
#define RAW_SAMPLES_PER_RECORD		(40 * MAXCHANNELS)

#define MESUREMENT_TYPE_MVC 		2
#define MESUREMENT_TYPE_ENDURANCE	3
#define COMMAND_BUF_SIZE 			4096
#define SERIAL_PORT_READ_ERROR      -1
#define SERIAL_PORT_BUF_OVERRUN 	-2




typedef struct
{
	uint32_t prefix;
	uint32_t command;
	uint32_t length;
}header;

typedef struct
{
	int numsamples;
	int elec[RAW_SAMPLES_PER_RECORD];

}elctrodes;

typedef struct
{
	uint8_t charging_status;
	uint8_t percent;
	uint32_t recharge_cycles;
}BATTERY_INFO;

typedef struct
{
	float rec;
	float rms_val[24];
	float avg_rms;
}RMS_INFO;

typedef struct
{
	float rms_val[24];
	float avg_rms;
}SERIAL_RMS_INFO;




typedef struct
{
	int min;
	int sec;
}time_info;


typedef enum
{
	change_bg_clr = 1,
	change_fg_clr ,
	change_bg_fg_clr
}BG_FG_STATUS;

RMS_INFO rms;

SERIAL_RMS_INFO serialrms;

//todo vmj DEBUG SETUP
bool debugSetup;


struct sockaddr_in servaddr;
header send_header,*recv_header;
ssize_t size;
elctrodes *elcs;
BATTERY_INFO battery;

float rawdata_mv[RAW_SAMPLES_PER_RECORD] = {0};
float filtered_output[RAW_SAMPLES_PER_RECORD] = {0};
int sockClient;

static int measure_diplay_window_samples =0;//measure_diplay_window_duration*10;

FILE *fp_data;
FILE *fp_summary;
FILE *fp_rawdata;

static char data_file[500] = "/home/ubuntu/MeasFiles/" ;
static char filename_summary[500] = "/home/ubuntu/MeasFiles/";
MYSQL *conn;
MYSQL_ROW row;
time_t rawtime,start_time,end_time;
double diff_time;
time_info start_time_con;
struct tm *timeinfo_rms,* timeinfo;
bool is_start_capture = false;
bool is_device_connected = false;

//veeresh
static int totSampleRecvd = 0;
char cont_start_time[100] = {0};
char cont_stop_time[100] = {0};
bool restTimeflag = false;
int activeTimecount ;
int restTimecount ;
int totTimecount;
int timeTopeak=0;
int timeTorest;
//int timeTomin;
float peakval;
float minval = 0;
//float minimum_val;
//bool rest_meas_start = false;
static int timeRemaining = 0;
float array_total_rms_value[25] = {0};
float array_cont_rms_value[25] = {0};
float array_cont_cmplt_smry[25] = { 0 };
float samplingperiod = 0.0;
float scale ;

float smallvl = 65536.0;

bool probelbl_updated = false;
bool contraction_flag; //Indicates contractiom state either active or rest
bool rms_stop_command_once;
int RMS_sample_count;
int count_before_con_strt;
static int cont_cycles_count = 0; //Which gives total contraction count
static int cont_count = 0;
extern int displayPointer;
extern int fillPointerLow;
extern int fillPointerHigh;



GtkWidget *statuslbl,*devicelbl,*connlbl,*probelbl,*probe_vallbl,*battlbl,*battvalllbl;
GtkWidget *measure_setlbl,*typelbl,*loclbl,*durationlbl,*restlbl,*actlbl,*cyclesllbl;
GtkWidget *measure_statlbl,*timelbl,*avglbl,*peaklbl,*onsetlbl,*offsetlbl;
GtkWidget *emg_statlbl,*cap_statlbl;
GtkWidget * vseparator;
GtkCssProvider *provider,*provider_win;
GtkStyleContext *context,*context_win,*ms_set_cntxt,*ms_stat_cntxt,*emg_stat_cntxt;
GtkWidget * levelBar ;

gchar connected_attr[] = "<span foreground='white' weight='bold' font='11'>  Connected</span>";
gchar disconnected_attr[] = "<span foreground='#183364' weight='bold' font='11'> Disconnected</span>";
gchar running_attr[] = "<span foreground='white' weight='bold' font='14'>Running - Active </span>";
gchar running_attr2[] = "<span foreground='white' weight='bold' font='12'>Running - Rest</span>";
gchar stopped_attr2[] = "<span foreground='white' weight='bold' font='11'>  Stopped</span>";
gchar probe_attr_30[] = "<span foreground='red' weight='bold' font='11'>  Low Battery</span>";
gchar probe_attr_31[] = "<span foreground='red' weight='bold' font='11'> Device on Charge</span>";
gchar probe_attr_34[] = "<span foreground='red' weight='bold' font='11'> Probe Disconnected</span>";
gchar probe_attr_35[] = "<span foreground='red' weight='bold' font='11'> Lead-off Fail</span>";


void displayXYScaleInit(int xWindowDurationSec, int yMilliVol);
void measurement_start(void);
void contraction_data_calculation(void);
void scale_fun(float ary[],float* scale_val,float dest[],int blk_size);
void mean_fun(float ary[],int size,float*dest);
void store_contraction_smry(void);
void calculate_running_avg(void);
void contraction_process(void);

void MeasBeep(char f);
void DisablePcSpkr();

static void load_data_to_display(SERIAL_RMS_INFO *rms_cb)
{

	cb.e1[fillPointerHigh] =(double)rms_cb->rms_val[0];
	cb.e2[fillPointerHigh] = (double)rms_cb->rms_val[1];
	cb.e3[fillPointerHigh] = (double)rms_cb->rms_val[2];
	cb.e4[fillPointerHigh] = (double)rms_cb->rms_val[3];
	cb.e5[fillPointerHigh] = (double)rms_cb->rms_val[4];
	cb.e6[fillPointerHigh] = (double)rms_cb->rms_val[5];
	cb.e7[fillPointerHigh] = (double)rms_cb->rms_val[6];
	cb.e8[fillPointerHigh] = (double)rms_cb->rms_val[7];
	cb.e9[fillPointerHigh] = (double)rms_cb->rms_val[8];
	cb.e10[fillPointerHigh] = (double)rms_cb->rms_val[9];
	cb.e11[fillPointerHigh] = (double)rms_cb->rms_val[10];
	cb.e12[fillPointerHigh] = (double)rms_cb->rms_val[11];
	cb.e13[fillPointerHigh] = (double)rms_cb->rms_val[12];
	cb.e14[fillPointerHigh] = (double)rms_cb->rms_val[13];
	cb.e15[fillPointerHigh] = (double)rms_cb->rms_val[14];
	cb.e16[fillPointerHigh] = (double)rms_cb->rms_val[15];
	cb.e17[fillPointerHigh] = (double)rms_cb->rms_val[16];
	cb.e18[fillPointerHigh] = (double)rms_cb->rms_val[17];
	cb.e19[fillPointerHigh] = (double)rms_cb->rms_val[18];
	cb.e20[fillPointerHigh] = (double)rms_cb->rms_val[19];
	cb.e21[fillPointerHigh] = (double)rms_cb->rms_val[20];
	cb.e22[fillPointerHigh] = (double)rms_cb->rms_val[21];
	cb.e23[fillPointerHigh] = (double)rms_cb->rms_val[22];
	cb.e24[fillPointerHigh] = (double)rms_cb->rms_val[23];
	cb.eavg[fillPointerHigh] = ((double)rms_cb->avg_rms*2);

	cb.e1[fillPointerLow] =(double)rms_cb->rms_val[0];
	cb.e2[fillPointerLow] = (double)rms_cb->rms_val[1];
	cb.e3[fillPointerLow] = (double)rms_cb->rms_val[2];
	cb.e4[fillPointerLow] = (double)rms_cb->rms_val[3];
	cb.e5[fillPointerLow] = (double)rms_cb->rms_val[4];
	cb.e6[fillPointerLow] = (double)rms_cb->rms_val[5];
	cb.e7[fillPointerLow] = (double)rms_cb->rms_val[6];
	cb.e8[fillPointerLow] = (double)rms_cb->rms_val[7];
	cb.e9[fillPointerLow] = (double)rms_cb->rms_val[8];
	cb.e10[fillPointerLow] = (double)rms_cb->rms_val[9];
	cb.e11[fillPointerLow] = (double)rms_cb->rms_val[10];
	cb.e12[fillPointerLow] = (double)rms_cb->rms_val[11];
	cb.e13[fillPointerLow] =(double)rms_cb->rms_val[12];
	cb.e14[fillPointerLow] = (double)rms_cb->rms_val[13];
	cb.e15[fillPointerLow] = (double)rms_cb->rms_val[14];
	cb.e16[fillPointerLow] = (double)rms_cb->rms_val[15];
	cb.e17[fillPointerLow] = (double)rms_cb->rms_val[16];
	cb.e18[fillPointerLow] = (double)rms_cb->rms_val[17];
	cb.e19[fillPointerLow] = (double)rms_cb->rms_val[18];
	cb.e20[fillPointerLow] = (double)rms_cb->rms_val[19];
	cb.e21[fillPointerLow] = (double)rms_cb->rms_val[20];
	cb.e22[fillPointerLow] = (double)rms_cb->rms_val[21];
	cb.e23[fillPointerLow] = (double)rms_cb->rms_val[22];
	cb.e24[fillPointerLow] = (double)rms_cb->rms_val[23];
	cb.eavg[fillPointerLow] = ((double)rms_cb->avg_rms*2);

	if((contraction_flag == false) && (meausrement_settings.measurement_type > 1))
	{
		cb.marker_val[fillPointerLow]= 0;
		cb.marker_val[fillPointerHigh] = 0;
	}
	else if((contraction_flag == true) && (meausrement_settings.measurement_type > 1))
	{
		cb.marker_val[fillPointerLow]= 500;
		cb.marker_val[fillPointerHigh]= 500;
	}


}

/*
bool firstTime = true;
static gboolean idle_callback()
{
	if(firstTime ==true){
		g_timeout_add (100,timer_callback,(gpointer)&cb);
		firstTime=false;
	}
	return true;
}
*/
int serial_port ;//= open("/dev/ttyUSB0", O_RDWR);

int interface_attibutes(int fd)
{
		struct termios tty;

		  // Read in existing settings, and handle any error
		  if(tcgetattr(serial_port, &tty) != 0)
		  {
		      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		      return 1;
		  }

		  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
		  tty.c_cflag |= CS8; // 8 bits per byte (most common)
		  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		  tty.c_lflag &= ~ICANON;
		  tty.c_lflag &= ~ECHO; // Disable echo
		  tty.c_lflag &= ~ECHOE; // Disable erasure
		  tty.c_lflag &= ~ECHONL; // Disable new-line echo
		  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
		  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		  tty.c_cc[VMIN] = 0;

		  // Set in/out baud rate to be 9600
		 //cfsetispeed(&tty, B9600);
		 //cfsetospeed(&tty, B9600);

		  cfsetispeed(&tty, B115200);
		  cfsetospeed(&tty, B115200);


		  // Save tty settings, also checking for error
		  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
		  {
		      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		      return 1;
		  }
return 0;
}


int interface_attibutes1(int fd)
{
	 struct termios tty;
	        if (tcgetattr (fd, &tty) != 0)
	        {
	        	 printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	               // error_message ("error %d from tcgetattr", errno);
	                return -1;
	        }

	        cfsetospeed (&tty, B115200);
	        cfsetispeed (&tty, B115200);

	        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	        // disable IGNBRK for mismatched speed tests; otherwise receive break
	        // as \000 chars
	        tty.c_iflag &= ~IGNBRK;         // disable break processing
	        tty.c_lflag = 0;                // no signaling chars, no echo,
	                                        // no canonical processing
	        tty.c_oflag = 0;                // no remapping, no delays
	        tty.c_cc[VMIN]  = 0;            // read doesn't block
	        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	                                        // enable reading
	        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	        //tty.c_cflag |= parity;
	        tty.c_cflag |= 0;
	        tty.c_cflag &= ~CSTOPB;
	        tty.c_cflag &= ~CRTSCTS;

	        if (tcsetattr (fd, TCSANOW, &tty) != 0)
	        {
	        	 printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	        //	error_message ("error %d from tcsetattr", errno);
	                return -1;
	        }
	        return 0;

}

int serialPortInit()
{
	// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
	  //int
	  serial_port = open("/dev/ttyACM0", O_RDWR);

	  // Create new termios struc, we call it 'tty' for convention

	  //interface_attibutes(serial_port);
	  interface_attibutes1(serial_port);
	  // Write to serial port
	  unsigned char msg[] = "Hello,world!\n";
	//  write(serial_port, msg, sizeof(msg));

	  // Allocate memory for read buffer, set size according to your needs
	  char read_buf [256];

	  // Normally you wouldn't do this memset() call, but since we will just receive
	  // ASCII data for this example, we'll set everything to 0 so we can
	  // call printf() easily.
	  memset(&read_buf, '\0', sizeof(read_buf));
	  set_blocking(serial_port,1);
	  // Read bytes. The behaviour of read() (e.g. does it block?,
	  // how long does it block for?) depends on the configuration
	  // settings above, specifically VMIN and VTIME
	  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

	  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
	  if (num_bytes < 0) {

		  printf("Error reading: %s", strerror(errno));
	      return 1;
	  }

	  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
	  // print it to the screen like this!)
	  printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);

	  //close(serial_port);
	  return 0; // success
}
void getTimestamp(char timestring[])
{
    struct tm gmtval    = {0};
    struct timespec curtime = {0};
    char timebuffer[20] = {0};
	time_t seconds = 0;
	long milliseconds = 0;

    // Get current time
    clock_gettime(CLOCK_REALTIME, &curtime);

    // Set the fields
    seconds      = curtime.tv_sec;
    milliseconds = (curtime.tv_nsec/1.0e6);

    if((localtime_r(&seconds, &gmtval)) != NULL)
    {
        // Build the first part of the time
        strftime(timebuffer, sizeof(timebuffer), "%Y-%m-%d %H:%M:%S", &gmtval);

        // Add the milliseconds part and build the time string
        snprintf(timestring, 40, "%s.%03ld", timebuffer,milliseconds);
    }
}

void calcTimestamp(void)
{
	char calc_timebuffer[20] = {0};
	static uint8_t calc_hr = 0;
	static uint8_t calc_min = 0;
	static uint8_t calc_sec = 0;
	static uint16_t calc_millisec = 0;

	calc_millisec += 100;
	if(calc_millisec >= 1000)
	{
		calc_millisec = 0;
		calc_sec += 1;

		if(calc_sec >= 60)
		{
			calc_sec  = 0;
			calc_min += 1;

			if(calc_min >= 60)
			{
				calc_min  = 0;
				calc_hr += 1;

			}
		}

	}
	sprintf(calc_timebuffer,"%02d:%02d:%02d.%03d",calc_hr,calc_min,calc_sec,calc_millisec);

	//fprintf (fp_data,"%s,",calc_timebuffer);
}

float MilliVoltConv(int adc_val)
{
    float millivolt = (float)((adc_val * 3.0) / 16777215.0) * 1000;
    return millivolt;

}

uint32_t DecodeInt32(uint32_t val1)
{
    uint32_t val = (uint32_t)(val1 << 24);
    val = val + (uint32_t)((val1 << 8 & 0xFF0000 ));
    val =  val + (uint32_t)((val1 >> 8 & 0x0000FF00));
    val = val + (uint32_t)(val1 >> 24 & 0xFF);
    return val;
}

unsigned long DecodChartoInt(char * buf)
{
    unsigned long val = (unsigned long)(buf[0] << 24);
    val = val + (unsigned long)(buf[1] << 16);
    val = val + (unsigned long)(buf[2] << 8);
    val = val + (unsigned long)(buf[3]);
    return val;
}

void EncodeInt32MSB(uint32_t value,unsigned char *encodedByteArray)
{
	encodedByteArray[0] = (char)(value >> 24 & 0xFF);
	encodedByteArray[1] = (unsigned char)(value >> 16 & 0xFF);
	encodedByteArray[2] = (unsigned char)(value >> 8 & 0xFF);
	encodedByteArray[3] = (unsigned char)(value & 0xFF);
}

void filter(float raw_input[],float filter_output[],int num_samples)
{
	HighPassFilter(raw_input, filter_output,num_samples);
	//printf("\n\nHPF Output in main\n\n");
	/*for(int i = 1; i <= num_samples;i++)
	{
		printf("%f,",filter_output[i-1]);
		if((i%24 == 0) && (i !=0)) printf("\n");
	}*/

	//Averaging
	Averaging(filter_output, filter_output,0xFFFFFF,num_samples);
	//printf("\n\nAvg Output\n\n");
	/*for(int i = 1; i <= num_samples;i++)
	{
		printf("%f,",filter_output[i-1]);
		if((i%24 == 0) && (i !=0)) printf("\n");
	}*/
	//Comb filter
	CombFilter(filter_output, filter_output,num_samples);
	//printf("\n\nComb Filter Output\n\n");
	for(int i = 1; i <= num_samples;i++)
	{
		//fprintf(fp_data,"\n%f,%f",raw_input[i-1],filter_output[i-1]);
		//printf("%f,",filter_output[i-1]);
		//if((i%24 == 0) && (i !=0)) printf("\n");
	}

}
void stringtofloat(char str[],float *dest,uint16_t *numoffields)
{
	char* arrayOfString[50] ;
	const char separator[2] = ", ";
	char *token;
	int nooffields = 0;
	float sum =0.0f;

	token = strtok(str, separator);

	while( token != NULL )
	{
	  arrayOfString[nooffields] = token;
	  token = strtok(NULL, separator);
	  nooffields++;
	}
	int m =0 ;
	for(int p =0 ; p < nooffields;p++)
	{
		dest[m] = (atof)(arrayOfString[p]);
		sum+=dest[m];
		m++;
	}
	*numoffields = nooffields;
	++m;
	//printf("Sum =%f,avg = %f, num fields=%d\n",sum,rms.avg_rms,nooffields);
}



//void stringToFloatConveration(char str[],float *dest,uint16_t *numoffields)
void stringToFloatConveration(char *str,float *dest,uint16_t *numoffields)
{


		const char separator[3] = ", ";
		//char *token;
		float data_array[25] ;
		int count = 0;
		int i = 0 ;
		//char str1[] = "953#1,0.2,0.2,0.0,0.1,0.2, 0.2, 0.0, 0.0, 0.2, 0.1, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.1";

		char *		token = strtok(str, separator);


		//token = strtok(str, separator);
		while(token != NULL)
		{
			if(i >= 1)
			{
				dest[i-1] = atof(token);
				//data_array[i] = atof(token);
				//printf("%lf\n",token[i]);
				count++;
			}
			token = strtok(NULL,separator);
			i++;
		}
		*numoffields = count;

/*
		char* arrayOfString[50] ;
		int nooffields = 0;
		float sum =0.0f;


		while( token != NULL )
		{
		  arrayOfString[nooffields] = token;
		  token = strtok(NULL, separator);
		  nooffields++;
		}
		int m = 0;
		for(int p = 0 ; p < nooffields;p++)
		{
			dest[m] = (atof)(arrayOfString[p]);
			sum+=dest[m];
			m++;
		}
		*numoffields = nooffields;
		++m;*/
		//printf("Sum =%f,avg = %f, num fields=%d\n",sum,rms.avg_rms,nooffields);


}


int RequestMeasStop(void)
{
	char buffer[256];
	char bufferPayload[256];
	char bufferRecv[256];

	gtk_widget_set_sensitive(x_adj_button,true);
	gtk_widget_set_sensitive(y_adj_button,true);

	printf("Req Measurement Stop\n");
	memset(bufferPayload,0,256);
	memset(buffer,0,256);
	memset(bufferRecv,0,256);

	send_header.prefix = PREFIX;
	send_header.command = REQ_MEAS_STOP;
	send_header.length = 0;
	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );

	memcpy(buffer, &send_header, HEADERLEN);

	int lenBuf = HEADERLEN;

	if(sockClient==0 ){
		printf("Not connected");
		return -1;
	}

	send(sockClient , buffer , lenBuf , 0 );

	return 1;

}

void close_window(void)
{
	char load[500] = "\0";
	char buf[5] = "\0";
	int ret = 0;

	RequestMeasStop();
	gtk_widget_set_sensitive(start_button,false);
	gtk_widget_set_sensitive(pause_button,false);
	gtk_widget_set_sensitive(save_button,false);

	if(is_start_capture == true)
	{
		time(&end_time);
		is_start_capture = false;
		diff_time = difftime(end_time,start_time);
		printf("Difference time in close window : %f\n",diff_time);
	}


	int status_pc = (int)((diff_time / meausrement_settings.measure_duration) * 100.0);
	if(status_pc > 100)
		status_pc = 100;
	sprintf(buf,"%d",status_pc);
	printf("Percent in close file: %s\n",buf);

	memset(load,0x00,sizeof(load));
	strcat(load,"UPDATE MeasurementPrescriptionMaster SET PercentageCompleted ='");
	strcat(load,buf);
	strcat(load,"' WHERE id = (select max(id) from MeasurementPrescriptionMaster);");


	printf("Percentage query: %s\n",load);

	if (mysql_query(conn,load))
	{
		printf("Failed to execute query. Error: %s\n", mysql_error(conn));
		//return 0;
	}

	ret = remove(data_file);

	if(meausrement_settings.measurement_type == 2)
		ret |= remove(filename_summary);

	if(ret == 0)
		printf("File deleted");
	else
		printf("Error in deleting");

	//gtk_main_quit();
	gtk_widget_destroy(window_main);

}

void save_file(void)
{
if(debugSetup == false )
{
	int ret = 0;

	char load[500] = "\0";

	gtk_widget_set_sensitive(start_button,false);
	gtk_widget_set_sensitive(pause_button,false);
	gtk_widget_set_sensitive(save_button,false);
	gtk_widget_set_sensitive(close_button,false);

	//if(is_start_capture == true)
	{
		is_start_capture = false;
		time(&end_time);
		diff_time = difftime(end_time,start_time);
		printf("Difference time in save file: %f\n",diff_time);
	}
	RequestMeasStop();

	ret = fclose(fp_data);
	ret = fclose(fp_summary);

	/*
	if(meausrement_settings.measurement_type == 2)
		ret |= fclose(fp_summary);
*/
	memset(load,0x00,sizeof(load));
	strcat(load,"UPDATE TempMeasurementSettings SET RMS_Data ='");
	strcat(load,data_file);
	strcat(load,"' WHERE id = (select max(id) from TempMeasurementSettings);");


	//printf("%s\n",load);

	if (mysql_query(conn,load))
	{
		printf("Failed to execute query. Error: %s\n", mysql_error(conn));
		//return 0;
	}

	char buf[5] = "\0";
	int status_pc = (int)((diff_time / meausrement_settings.measure_duration) * 100.0);
	if(status_pc > 100)
		status_pc = 100;
	sprintf(buf,"%d",status_pc);

	memset(load,0x00,sizeof(load));
	strcat(load,"UPDATE MeasurementPrescriptionMaster SET PercentageCompleted ='");
	strcat(load,buf);
	strcat(load,"' WHERE id = (select max(id) from MeasurementPrescriptionMaster);");


	printf("Percentage query: %s\n",load);

	if (mysql_query(conn,load))
	{
		printf("Failed to execute query. Error: %s\n", mysql_error(conn));
		//return 0;
	}


	//if(meausrement_settings.measurement_type == 2)
	{
		memset(load,0x00,sizeof(load));
		strcat(load,"UPDATE TempMeasurementSettings SET Summary ='");
		strcat(load,filename_summary);
		strcat(load,"' WHERE id = (select max(id) from TempMeasurementSettings);");


		//printf("%s\n",load);

		if (mysql_query(conn,load))
		{
			printf("Failed to execute query. Error: %s\n", mysql_error(conn));
			//return 0;
		}
	}

	if(ret == 0)
		printf("File closed");
	else
		printf("Error in closing");

	printf("executing python script");
	if(system("python3 /home/ubuntu/yash/Research_system/proto.py &> /home/ubuntu/bmjo/test.out")==-1)
	{
		printf("failed to run");  //prints !!!Hello World!!!
	}

}
else if(debugSetup == true)
{
//send measurement stop command

	TxCommand_stop();
}

	GtkWidget *dialog;
	dialog = gtk_message_dialog_new(GTK_WINDOW(window_main),
										GTK_DIALOG_DESTROY_WITH_PARENT,
										GTK_MESSAGE_INFO,
										GTK_BUTTONS_CLOSE,
										"Saved Successfully");
	gtk_dialog_run(GTK_DIALOG(dialog));
	gtk_widget_destroy(dialog);

	gtk_widget_destroy(window_main);

}

int RequestSetFilterFreq()
{
	char buffer[50];
	char bufferRecv[256];
	char bufferPaylod[10];
	int payloadLen;

	printf("Set Freq\n");
	memset(buffer,0,sizeof(buffer));
	memset(bufferRecv,0,sizeof(bufferRecv));
	memset(bufferPaylod,0,sizeof(bufferPaylod));

	sprintf(bufferPaylod,"%d",FILTER_FREQ);
	payloadLen = strlen(bufferPaylod);
	printf("%s",bufferPaylod);

	send_header.prefix = PREFIX;
	send_header.command = REQ_SETFILTERFREQ;
	send_header.length = payloadLen;

	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );

	memcpy(buffer, &send_header, HEADERLEN);
	memcpy((buffer+HEADERLEN), bufferPaylod, payloadLen);
	int lenBuf = HEADERLEN + payloadLen;

	if(sockClient==0 )
	{
		printf("Not connected");
		return -1;
	}

	if(send(sockClient , buffer , lenBuf , 0 ) < 0)
	{
		printf("send failed");
		return -1;

	}

	/*if( (size = recv(sockClient , bufferRecv , 256 , 0)) < 0)
	{
		fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
		printf("recv failed");
		return -1;

	}
*/	//printf("Resp : %s\n", &bufferRecv[12]);

	return 1;

}


int RequestMeasStart_Raw()
{
	char buffer[512];
	char bufferRecv[512];
	char bufferPaylod[512];
	int payloadLen;

	printf("Raw Data Measurement Start\n");
	memset(buffer,0,512);
	memset(bufferRecv,0,512);

	sprintf(bufferPaylod,"%s,%d,%d,%d,%d,%d,%d,%d,%d,%d",ENABLE_CHANNELS,INPUT_SAMPLERATE,MAXTIME,MEAS_TYPE_RAW,REFERENCE_GROUP,CMP_REFERENCE_GROUP,OUTPUT_SAMPLERATE,INTIALIZATION_TIME,ACTIVE_TIME,REST_TIME);
	payloadLen = strlen(bufferPaylod);
	printf("%s",bufferPaylod);

	send_header.prefix = PREFIX;
	send_header.command = REQ_MEAS_START;
	send_header.length = payloadLen;

	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );


	memcpy(buffer, &send_header, HEADERLEN);
	memcpy((buffer+HEADERLEN), bufferPaylod, payloadLen);
	int lenBuf = HEADERLEN + payloadLen;

	if(sockClient==0 )
	{
		printf("Not connected");
		return -1;
	}

	if(send(sockClient , buffer , lenBuf , 0 ) < 0)
	{
		printf("send failed");
		return -1;

	}

/*
	int i = 0;
	while(i < 1000)
	{
		++i;
		if( (size = recv(sockClient , bufferRecv , 512 , 0)) < 0)
		{
			fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
			printf("recv failed");
			//return -1;

		}
		printf("Resp : %s\n", &bufferRecv[12]);
	}
*/
		//printf("IP addr: %s\n", inet_ntoa(servaddr.sin_addr));


	return 1;

}

int RequestMeasStart_RMS()
{
	int xRange;
	int yRange;

	if(is_start_capture == false)
	{
		xRange = (int)gtk_spin_button_get_value(GTK_SPIN_BUTTON(x_adj_button));
		yRange = (int)gtk_spin_button_get_value(GTK_SPIN_BUTTON(y_adj_button));
		displayXYScaleInit(xRange,yRange);
		probelbl_updated = false;
	}
	gtk_widget_set_sensitive(x_adj_button,false);
	gtk_widget_set_sensitive(y_adj_button,false);

	printf("Req Start Measurement\n");

	if(debugSetup == false )
	{
	//printf("%f\n",xy_axis_range.x_max_range);
	//printf("%f\n",xy_axis_range.y_max_range);
	//MEASUREMENT_SETUP measurement_info;
	char buffer[512];
	char bufferRecv[512];
	char bufferPaylod[512];
	int payloadLen;

	//gtk_widget_set_sensitive(start_button,false);
	//gtk_widget_set_sensitive(x_adj_button,false);
	//gtk_widget_set_sensitive(y_adj_button,false);

	printf("Req Start Measurement\n");
	memset(buffer,0,512);
	memset(bufferRecv,0,512);

	//printf("MT: %d\n", measurement_type);

	//if(meausrement_settings.measure_duration<=0)meausrement_settings.measure_duration=60; // ASK VEERESH
	sprintf(bufferPaylod,"%s,%d,%d,%d,%d,%d,%d,%d,%d,%d",ENABLE_CHANNELS,INPUT_SAMPLERATE,timeRemaining,meausrement_settings.measurement_type,REFERENCE_GROUP,CMP_REFERENCE_GROUP,OUTPUT_SAMPLERATE,INTIALIZATION_TIME,meausrement_settings.active_time,meausrement_settings.rest_time);
	payloadLen = strlen(bufferPaylod);
	printf("%s",bufferPaylod);

	send_header.prefix = PREFIX;
	send_header.command = REQ_MEAS_START;
	send_header.length = payloadLen;

	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );


	memcpy(buffer, &send_header, HEADERLEN);
	memcpy((buffer+HEADERLEN), bufferPaylod, payloadLen);
	int lenBuf = HEADERLEN + payloadLen;


	if(sockClient==0 )
	{
		printf("Not connected");
		return -1;
	}

	if(send(sockClient , buffer , lenBuf , 0 ) < 0)
	{
		printf("send failed");
		return -1;

	}
/*	int i=0;
	while(i<1000)
	{
		++i;
		if( (size = recv(sockClient , bufferRecv , 512 , 0)) < 0)
		{
			fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
			printf("recv failed");
			//return -1;

		}
		printf("Resp : %s\n", &bufferRecv[12]);
	}
	//printf("IP addr: %s\n", inet_ntoa(servaddr.sin_addr));
*/

	}

	else if(debugSetup == true)
	{
		//TxCommand();//Send measurement start command via serial port
	}
	return 1;
}

int RequestInfoBattery()
{
	char buffer[256];
	char bufferPayload[256];
	char bufferRecv[256];//

	printf("Battery Info\n");
	memset(bufferPayload,0,256);
	memset(buffer,0,256);
	memset(bufferRecv,0,256);

	send_header.prefix = PREFIX;
	send_header.command = REQ_INFO_BATTERY;
	send_header.length = 0;
	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );

	memcpy(buffer, &send_header, HEADERLEN);

	int lenBuf = HEADERLEN;

	if(sockClient==0 ){
		printf("Not connected");
		return -1;
	}

	send(sockClient , buffer , lenBuf , 0 );

	/*if( recv(sockClient , bufferRecv , 256 , 0) < 0)
	{
		printf("recv failed");
		return -1;

	}
	printf("Battery response : 0 -Success -1 failed - 2 unkniwn error --->%s\n", &bufferRecv[12]);
	//printf("IP addr: %s\n", inet_ntoa(servaddr.sin_addr));*/

	return 1;

}
int ConnectToDev()
{
	char buffer[256];
	char bufferPayload[256];
	char bufferRecv[256];
	int payloadLen;
	//struct sockaddr_in servaddr;

	gtk_widget_set_sensitive(start_button,false);
	gtk_widget_set_sensitive(pause_button,false);
	gtk_widget_set_sensitive(save_button,false);

	memset(bufferPayload,0,256);
	memset(buffer,0,256);
	memset(bufferRecv,0,256);


	sprintf(bufferPayload,"%s,%d\n","00:07:80:01:91:a2",(int)time(NULL));
	payloadLen = strlen(bufferPayload);

	send_header.prefix = PREFIX;
	send_header.command = REQ_CONNECT;
	send_header.length = payloadLen;
	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );

	memcpy(buffer, &send_header, HEADERLEN);
	memcpy(buffer + HEADERLEN, bufferPayload, payloadLen);
	int lenBuf = HEADERLEN + payloadLen;

	//servaddr.sin_family = AF_INET;
	//servaddr.sin_port = htons(PORT);

	if ((sockClient = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0)
	{
		perror("socket creation failed");
		exit(EXIT_FAILURE);
		is_device_connected = false;
		return -1;
	}

	/*if(inet_pton(AF_INET, "192.168.0.101", &servaddr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}*/

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if (setsockopt(sockClient, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof tv))
	{
		printf("Failed to set timeout");
		return -4;
	}

	if (connect(sockClient, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
	{
		printf("\nConnection Failed \n");
		is_device_connected = false;
		return -1;
	}

	send(sockClient , buffer , lenBuf , 0 );

	if( recv(sockClient , bufferRecv , 256 , 0) < 0)
	{
		printf("recv failed\n");
		is_device_connected = false;
		//return -1;

	}

	printf("Connect response : 0 -Success -1 failed - 2 unkniwn error --->%s\n", &bufferRecv[12]);
	printf("IP addr: %s\n", inet_ntoa(servaddr.sin_addr));


	if(bufferRecv[12] == '0')
	{
		gtk_label_set_markup (GTK_LABEL(connlbl), connected_attr);
		probelbl_updated = true;
		is_device_connected = true;
		gtk_widget_set_sensitive(reconnect_button,false);
		gtk_widget_set_sensitive(start_button,true);
		gtk_widget_set_sensitive(pause_button,true);
		gtk_widget_set_sensitive(save_button,true);

	}


	return 1;

}
//1- Success
//-1 - Failiure to init socket
//-2 - Braodcast not permited
//-3 - Failed to send request
//-4 - Failed to set time out
//-5 - Failed to recv response
int RequestServDisc()
{
	int sockfd;
	char buffer[256];
	char bufferRecv[256];
	//struct sockaddr_in servaddr;

	// Creating socket file descriptor
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		perror("socket creation failed");
		exit(EXIT_FAILURE);
		return -1;
	}

	int  broadcastPermission = 1;
	if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0)
	{
		printf("setsockopt() failed->%d",errno);
		return -1;
	}
	else printf("Broadcast permited\n");

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof tv)) {
		printf("Failed to set timeout");
		return -4;
	}

	send_header.prefix = PREFIX;
	send_header.command = REQ_DISCOVERY;
	send_header.length = CLIENTMACADR_SIZE;
	EncodeInt32MSB(send_header.command,(unsigned char *) &send_header.command );
	EncodeInt32MSB(send_header.length,(unsigned char *) &send_header.length );

	memcpy(buffer, &send_header, HEADERLEN);
	memcpy(buffer + HEADERLEN, CLIENTMACADR, CLIENTMACADR_SIZE);
	int lenBuf = HEADERLEN + CLIENTMACADR_SIZE;

	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(3000);
	servaddr.sin_addr.s_addr = inet_addr(BRAODCAST_IP_RANGE);// htonl(INADDR_ANY);//inet_addr("255.255.255.255");//

	if ((sendto(sockfd, buffer, lenBuf, 0, (const struct sockaddr*) & servaddr, sizeof(servaddr))) < 0)
	{
	 printf("Message send fail-->%d\r\n", errno);
	 return -3;
	}
	else printf("Message sent\n");

	socklen_t len = sizeof(struct sockaddr_in);
	if(recvfrom(sockfd, bufferRecv, 256, 0, (struct sockaddr*)&servaddr, &len)<0)
	{
		printf("Receive failed");
		return -5; //recv failed
	}
	printf("Server response-->%s\n", &bufferRecv[12]);
	//printf("IP addr: %s\n", inet_ntoa(servaddr.sin_addr));

	close(sockfd);
	return 1;

}

void disp_update_device_status(void)
{
	char bt_pc[5] = {0};
	gchar low_battery_attr[100] = {0};
	gtk_label_set_markup (GTK_LABEL(connlbl), connected_attr);

	sprintf(bt_pc,"%d",battery.percent);
	strcat(bt_pc,"%");

	if(battery.percent > 5)
	{
		gtk_label_set_text(GTK_LABEL(battvalllbl),bt_pc);
	}
	else
	{
		strcpy(low_battery_attr,"<span foreground='red' weight='bold' font='11'> ");
		strcat(low_battery_attr , bt_pc);
		strcat(low_battery_attr ," - Low Battery</span>");
		gtk_label_set_markup (GTK_LABEL(battvalllbl), low_battery_attr);
	}

	gtk_level_bar_set_value(GTK_LEVEL_BAR(levelBar),(gdouble)battery.percent);



}

void Controlbar_init(void)
{
	GtkAdjustment *adjustment_x,*adjustment_y;

	//GtkWidget *reconnect_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/play.png");
	reconnect_button = gtk_button_new_with_label("Reconnect");
	//gtk_button_set_image(GTK_BUTTON(reconnect_button),reconnect_image);
	gtk_widget_set_margin_end(reconnect_button,30);
	gtk_widget_set_focus_on_click(reconnect_button,false);


	switch_eleclbl = gtk_label_new("Switch Electrodes");
	elec_switcher = gtk_switch_new ();
	gtk_widget_set_halign(elec_switcher,GTK_ALIGN_START);
	gtk_widget_set_margin_end(elec_switcher,30);

	GtkWidget *play_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/play.png");
	start_button = gtk_button_new();
	gtk_button_set_image(GTK_BUTTON(start_button),play_image);
	gtk_widget_set_focus_on_click(start_button,true);


	GtkWidget *pause_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/pause.png");
	pause_button = gtk_button_new();
	gtk_button_set_image(GTK_BUTTON(pause_button),pause_image);
	gtk_widget_set_focus_on_click(pause_button,true);

	save_button = gtk_button_new();
	GtkWidget *save_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/save.png");
	gtk_button_set_image(GTK_BUTTON(save_button),save_image);
	gtk_widget_set_focus_on_click(save_button,true);

	close_button = gtk_button_new();
	GtkWidget *cancel_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/cancel.png");
	gtk_button_set_image(GTK_BUTTON(close_button),cancel_image);
	gtk_widget_set_focus_on_click(close_button,true);
	gtk_widget_set_margin_end(close_button,30);

	adjustment_x = gtk_adjustment_new (meausrement_settings.emg_timespan, 5.0, 60.0, 5.0, 5.0, 0.0);
	adjustment_y = gtk_adjustment_new (meausrement_settings.emg_sensitivity, 5.0, 200.0, 5.0, 5.0, 0.0);


	label2 = gtk_label_new("X-axis");
	x_adj_button = gtk_spin_button_new(adjustment_x, 1.0, 0);

	label3 = gtk_label_new("Y-axis");
	y_adj_button = gtk_spin_button_new(adjustment_y, 1.0, 0);


}

void DeviceStstusDisplay_init(void)
{
	//GtkWidget *logo_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/test/logo.jpg");

	GtkWidget *logo_image = gtk_image_new_from_file ("/home/ubuntu/eclipse-workspace/ResearchSystemTestcode/logo.jpg");
	gtk_widget_set_margin_top(logo_image,20);

	statuslbl = gtk_label_new("STATUS");
	gtk_label_set_xalign (GTK_LABEL(statuslbl),0.1);
	gtk_widget_set_margin_top(statuslbl,20);
	gchar statuslbl_attr[] = "<span foreground='black' weight='bold' font='13'> STATUS </span>";
	gtk_label_set_markup (GTK_LABEL(statuslbl), statuslbl_attr);

	//Background colour for heading
	provider = gtk_css_provider_new ();
	context = gtk_widget_get_style_context (statuslbl);
	gtk_css_provider_load_from_data (provider, "label {background-color: #AAAAAA;}", -1, NULL);
	gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);


	devicelbl = gtk_label_new(" Device:");
	connlbl = gtk_label_new("Disconnected");
	probelbl= gtk_label_new(" Probe:");
	probe_vallbl = gtk_label_new("    ");
	battlbl = gtk_label_new("Battery");
	battvalllbl = gtk_label_new(" ");

	levelBar = gtk_level_bar_new_for_interval(0.0,100.0);
	gtk_level_bar_set_mode(GTK_LEVEL_BAR(levelBar),GTK_LEVEL_BAR_MODE_CONTINUOUS);
	gtk_widget_set_size_request(levelBar,5,15);
	gtk_widget_set_margin_start(levelBar,20);
	gtk_widget_set_margin_end(levelBar,40);
	//gtk_level_bar_set_value(GTK_LEVEL_BAR(levelBar),60.0);

	gchar disconnected_attr[] = "<span foreground='red' weight='bold' font='11'>  Disconnected</span>";
	gtk_label_set_markup (GTK_LABEL(connlbl), disconnected_attr);

	gtk_box_pack_start(GTK_BOX(notify_box), logo_image, FALSE, FALSE, 0);
	gtk_box_pack_start(GTK_BOX(notify_box), statuslbl, FALSE, TRUE, 10);
	gtk_box_pack_start(GTK_BOX(notify_box), devicelbl, FALSE, TRUE, 2);
	gtk_box_pack_start(GTK_BOX(notify_box), connlbl, FALSE, TRUE, 5);
	gtk_box_pack_start(GTK_BOX(notify_box), probelbl, FALSE, TRUE, 2);
	gtk_box_pack_start(GTK_BOX(notify_box), probe_vallbl, FALSE, TRUE, 5);
	gtk_box_pack_start(GTK_BOX(notify_box), battlbl, FALSE, FALSE, 2);
	gtk_box_pack_start(GTK_BOX(notify_box), levelBar, FALSE, FALSE,5);
	gtk_box_pack_start(GTK_BOX(notify_box), battvalllbl, FALSE, FALSE,0);

}

void MeasmntSettingsDisplay_init(void)
{
	measure_setlbl = gtk_label_new("MEASUREMENT SETTINGS");  //to be made bold
	gtk_label_set_xalign (GTK_LABEL(measure_setlbl),0.1);
	gchar measure_setlbl_attr[] = "<span foreground='black' weight='bold' font='13'> MEASUREMENT SETTINGS </span>";
	gtk_label_set_markup (GTK_LABEL(measure_setlbl), measure_setlbl_attr);

	ms_set_cntxt = gtk_widget_get_style_context (measure_setlbl);
	gtk_style_context_add_provider(ms_set_cntxt, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);


	char meas_txt[] 	= "   Type                   :  ";
	char loc_txt[]  	= "   Location          :  ";
	char dur_txt[]  	= " Duration (s)   :  ";
	char rest_txt[] 	= "Rest (s)             :  ";
	char active_txt[]  	= "Active (s)         :  ";
	char cycles_txt[]  	= "Cycles                :  ";
	char dur_char[4];
	strcat(meas_txt,"MVC");
	strcat(loc_txt,"Anal");
	sprintf(dur_char,"%d",meausrement_settings.measure_duration);
	strcat(dur_txt,"100");
	strcat(rest_txt,"2");
	strcat(active_txt,"3");
	strcat(cycles_txt,"20");

	typelbl = gtk_label_new(meas_txt);
	loclbl = gtk_label_new(loc_txt);
	durationlbl= gtk_label_new(dur_txt);
	restlbl = gtk_label_new(rest_txt);
	actlbl = gtk_label_new(active_txt);
	cyclesllbl = gtk_label_new(cycles_txt);

	gtk_label_set_xalign (GTK_LABEL(typelbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(loclbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(durationlbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(restlbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(actlbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(cyclesllbl),0.2);

	gtk_box_pack_start(GTK_BOX(notify_box), measure_setlbl, FALSE, TRUE, 20);
	gtk_box_pack_start(GTK_BOX(notify_box), typelbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), loclbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), durationlbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), restlbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), actlbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), cyclesllbl, FALSE, TRUE, 1);

}

void MeasmntStatusDisplay_init(void)
{
	measure_statlbl = gtk_label_new("MEASUREMENT STATUS");  //to be made bold
	gtk_label_set_xalign (GTK_LABEL(measure_statlbl),0.1);
	gchar measure_statlbl_attr[] = "<span foreground='black' weight='bold' font='13'> MEASUREMENT STATUS</span>";
	gtk_label_set_markup (GTK_LABEL(measure_statlbl), measure_statlbl_attr);


	ms_stat_cntxt = gtk_widget_get_style_context (measure_statlbl);
	gtk_style_context_add_provider(ms_stat_cntxt, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);


	timelbl   = gtk_label_new("Time            : ");
	avglbl    = gtk_label_new("Average     : ");
	peaklbl   = gtk_label_new("Peak            : ");
	onsetlbl  = gtk_label_new("Onset          : ");
	offsetlbl = gtk_label_new("Offset         : ");

	gtk_label_set_xalign (GTK_LABEL(timelbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(avglbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(peaklbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(onsetlbl),0.2);
	gtk_label_set_xalign (GTK_LABEL(offsetlbl),0.2);


	emg_statlbl = gtk_label_new("EMG STATUS");
	gtk_label_set_xalign (GTK_LABEL(emg_statlbl),0.1);
	gchar emg_statlbl_attr[] = "<span foreground='black' weight='bold' font='13'> EMG STATUS</span>";
	gtk_label_set_markup (GTK_LABEL(emg_statlbl), emg_statlbl_attr);
	emg_stat_cntxt = gtk_widget_get_style_context (emg_statlbl);
	gtk_style_context_add_provider(emg_stat_cntxt, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);


	cap_statlbl   = gtk_label_new("  ");

	gtk_box_pack_start(GTK_BOX(notify_box), measure_statlbl, FALSE, TRUE, 20);
	gtk_box_pack_start(GTK_BOX(notify_box), timelbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), avglbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), peaklbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), onsetlbl, FALSE, TRUE, 1);
	gtk_box_pack_start(GTK_BOX(notify_box), offsetlbl, FALSE, TRUE, 1);

	gtk_box_pack_start(GTK_BOX(notify_box), emg_statlbl, FALSE, TRUE, 20);
	gtk_box_pack_start(GTK_BOX(notify_box), cap_statlbl, FALSE, TRUE, 1);
}

//Veeresh fun add for colour schem change

void Modify_bg_fg_color(GtkWidget * label,BG_FG_STATUS status,char* bg_color_val,char* fg_color_val)
{
	GdkColor bgcolor,fgcolor;
	//GdkRGBA bgcolor1;

	if(status == change_bg_clr)
	{
		gdk_color_parse(bg_color_val,&bgcolor);
		//gdk_rgba_parse(&bgcolor1,bgcolor);

		gtk_widget_modify_bg(GTK_WIDGET(label),GTK_STATE_NORMAL,&bgcolor);
	}
	else if(status == change_fg_clr)
	{
		gdk_color_parse(fg_color_val,&fgcolor);
		gtk_widget_modify_fg(GTK_WIDGET(label),GTK_STATE_NORMAL,&fgcolor);
	}
	else if(status == change_bg_fg_clr)
	{


		gdk_color_parse(bg_color_val,&bgcolor);
		gdk_color_parse(fg_color_val,&fgcolor);

		gtk_widget_modify_bg(GTK_WIDGET(label),GTK_STATE_NORMAL,&bgcolor);
		gtk_widget_modify_fg(GTK_WIDGET(label),GTK_STATE_NORMAL,&fgcolor);
	}
}

void update_bg_fg_colors(void)
{
	//widgets background and forground changes
	Modify_bg_fg_color(notify_box,change_bg_clr,"#007bff",NULL);
	Modify_bg_fg_color(primary_box,change_bg_clr,"#f8f9fa",NULL);
	Modify_bg_fg_color(switch_eleclbl,change_bg_fg_clr,"#007bff","white");
	Modify_bg_fg_color(label2,change_bg_fg_clr,"#007bff","white");
	Modify_bg_fg_color(label3,change_bg_fg_clr,"#007bff","white");
	Modify_bg_fg_color(statuslbl,change_bg_fg_clr,"#62A1DC","red");
	Modify_bg_fg_color(typelbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(loclbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(durationlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(restlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(actlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(cyclesllbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(devicelbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(probelbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(battlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(battvalllbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(measure_setlbl,change_bg_clr,"#62A1DC",NULL);
	Modify_bg_fg_color(measure_statlbl,change_bg_clr,"#62A1DC",NULL);
	Modify_bg_fg_color(timelbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(avglbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(peaklbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(onsetlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(offsetlbl,change_bg_fg_clr,"#153D85","white");
	Modify_bg_fg_color(emg_statlbl,change_bg_clr,"#62A1DC",NULL);
}

void reconnect_device(void)
{
	if (is_device_connected == false )
	{
		RequestServDisc();
		ConnectToDev();
	}
}

//todo Rashmi - split this function for each group such as Play control bar, DeviceStstusDisplay, Measurement settings, MeasurementStatus
void window_init(void)
{

	window_main = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_widget_set_size_request(window_main ,1920,1020);
	//gtk_widget_set_size_request(window_main ,1024,768);

	gtk_window_fullscreen (GTK_WINDOW(gtk_widget_get_root_window (window_main)));

	primary_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);

	notify_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
	//gtk_widget_set_size_request(notify_box ,0,680);
	//gtk_widget_set_margin_top(notify_box,00);

	vseparator = gtk_separator_new (GTK_ORIENTATION_VERTICAL);


	box_for_graph = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
	gtk_widget_set_margin_top(box_for_graph,20);

	button_box1 = gtk_button_box_new (GTK_ORIENTATION_HORIZONTAL);
	gtk_button_box_set_layout (GTK_BUTTON_BOX (button_box1), GTK_BUTTONBOX_CENTER);
	gtk_box_set_spacing(GTK_BOX(button_box1), 10);
	gtk_widget_set_margin_bottom(button_box1,40);



	Controlbar_init();

	g_signal_connect(G_OBJECT(window_main), "destroy", G_CALLBACK(gtk_main_quit), NULL);
	g_signal_connect(G_OBJECT(reconnect_button), "clicked", G_CALLBACK(reconnect_device), NULL);
	g_signal_connect(G_OBJECT(start_button), "clicked", G_CALLBACK(RequestMeasStart_RMS), NULL);
	g_signal_connect(G_OBJECT(pause_button), "clicked", G_CALLBACK(RequestMeasStop), NULL);
	g_signal_connect(G_OBJECT(save_button), "clicked", G_CALLBACK(save_file), NULL);
	g_signal_connect(G_OBJECT(close_button), "clicked", G_CALLBACK(close_window), NULL);

	gtk_container_add(GTK_CONTAINER(window_main), primary_box);
	gtk_box_pack_start(GTK_BOX(primary_box), notify_box, FALSE,FALSE ,0);
	gtk_box_pack_start(GTK_BOX(primary_box), vseparator, FALSE,FALSE , 0);
	gtk_box_pack_start(GTK_BOX(primary_box), box_for_graph, TRUE,TRUE, 0);


	DeviceStstusDisplay_init();
	MeasmntSettingsDisplay_init();
	MeasmntStatusDisplay_init();

	update_bg_fg_colors();

}

void UpdateStatusLabel(bool running, bool con_statlblToggle)
{
	char disp_sum_buf[25];

	if(running)
	{
		if(con_statlblToggle == true)
		{
				gtk_label_set_markup (GTK_LABEL(cap_statlbl), running_attr);
				//labelToggle=false;
		}
		else if(con_statlblToggle == false)
		{
			gtk_label_set_markup (GTK_LABEL(cap_statlbl), running_attr2);

			//labelToggle = true;
		}
		if(meausrement_settings.measurement_type == 1)
		{
			sprintf(disp_sum_buf," Time            : %d",timeRemaining);
		}
		else
		{
			sprintf(disp_sum_buf,"  Time            : %d-%d",cont_count,timeRemaining);
		}
		gtk_label_set_text(GTK_LABEL(timelbl),disp_sum_buf);
		--timeRemaining;
		printf("timeRemaining: %d\n",timeRemaining);


		//if(rest_meas_start == true)
		//	gtk_label_set_markup (GTK_LABEL(connlbl), connected_attr);

	}
	else
	{
		//gtk_label_set_markup (GTK_LABEL(connlbl), stopped_attr2);
		gtk_label_set_markup (GTK_LABEL(cap_statlbl), stopped_attr2);
	}
}
void SerialDataParsing(char * bufferRecv ,bool simulation)
{
	char tmp[100] = {0};
		uint16_t numfieldsconv = 0;
		totSampleRecvd++;

		if(!simulation)
		{
			//printf("RMS Data: %s\r\n",bufferRecv);
			stringToFloatConveration(&bufferRecv[0],(float *)&serialrms,&numfieldsconv);
		}
printf("RMS data:");
for(int i=0;i<24;i++)
{
	serialrms.rms_val[i] = serialrms.rms_val[i]*4;
	printf("%f\t",serialrms.rms_val[i]);
}


		//getTimestamp(tmp);
		//calcTimestamp();
		/*int sampleNumber;
		int nRandonNumber = rand()%((200+1)-2) + 100;
		float curVal = sampleNumber%10 *2+nRandonNumber;
		sampleNumber++;
		for(int i=0;i<24;++i)
		{
			rms.rms_val[i] = curVal;

		}
		rms.avg_rms=curVal*2;
*/


		load_data_to_display(&serialrms);
		timer_callback(&cb);

		++fillPointerHigh;
		++fillPointerLow;

		//printf("Fill pointer in fill : %d\n",fillPointerLow);

		if(fillPointerLow >= xy_axis_range.x_max_range)
		{
			fillPointerLow = 0;
			fillPointerHigh = xy_axis_range.x_max_range;
		}

		if(fillPointerHigh%10==0)
		{
			UpdateStatusLabel(true,contraction_flag);
		}


}
void ProcessRMSData(char * bufferRecv,bool simulation)
{
	char tmp[100] = {0};
	uint16_t numfieldsconv = 0;
	totSampleRecvd++;
	//printf("totSampleRecvd : %d\n ",totSampleRecvd);

	if(!simulation)
	{
		//printf("RMS Data: %s\r\n",bufferRecv);
		stringtofloat(bufferRecv,(float *)&rms,&numfieldsconv);
	}
	else
	{
		if(meausrement_settings.measurement_type > 1)
		{
			if((!contraction_flag)&&(totSampleRecvd % (meausrement_settings.rest_timecount) == 0))
			{
				contraction_flag = true;
				cont_cycles_count++;
				printf("CONT ST\n");
				time(&rawtime);
				timeinfo =localtime( &rawtime );
				start_time_con.min = timeinfo->tm_min;
				start_time_con.sec = timeinfo->tm_sec;
				UpdateStatusLabel(true,contraction_flag);
				printf("cont_cycles_count : %d\n",cont_cycles_count);
			}
		}
	}

	//BMJO Todo keep it as a sperate func and call if the type is MVC or Endurance
	if(meausrement_settings.measurement_type > 1)
	{
		contraction_process();
	}
	else
	{
		calculate_running_avg();
	}


	//printf("average = %f\n",rms.avg_rms);
	for(int i = 0;i < (numfieldsconv-2); i++)
	{
			fprintf(fp_data,"%f,",rms.rms_val[i]);
			//printf ( "rms_val[%d]:  %f",i,rms.rms_val[i] );
	}

	fprintf(fp_data,"%f,",rms.avg_rms);
	//printf ( "avg_rms: %f",rms.avg_rms );


	getTimestamp(tmp);
	//printf ( "Current local time: %s",tmp );
	fprintf (fp_data,"%s,", tmp);

	//calcTimestamp();

	load_data_to_display(&rms);
	timer_callback(&cb);

	++fillPointerHigh;
	++fillPointerLow;

	//printf("Fill pointer in fill : %d\n",fillPointerLow);

	if(fillPointerLow >= xy_axis_range.x_max_range)
	{
		fillPointerLow = 0;
		fillPointerHigh = xy_axis_range.x_max_range;
	}

	if(fillPointerHigh%10==0)
	{
		UpdateStatusLabel(true,contraction_flag);
	}

}


bool UpdateProbelabel(int probestatus)
{

	switch(probestatus)
	{

		case 0:
			gtk_label_set_markup (GTK_LABEL(probe_vallbl), connected_attr);
			probelbl_updated = true;
			break;
		case -30:
			gtk_label_set_markup (GTK_LABEL(probe_vallbl), probe_attr_30);
			probelbl_updated = true;
			break;
		case -31:
			gtk_label_set_markup (GTK_LABEL(probe_vallbl), probe_attr_31);
			probelbl_updated = true;
			break;
		case -34:
			gtk_label_set_markup (GTK_LABEL(probe_vallbl), probe_attr_34);
			probelbl_updated = true;
			break;
		case -35:
			gtk_label_set_markup (GTK_LABEL(probe_vallbl), probe_attr_35);
			probelbl_updated = true;
			break;

		default:
			break;

	}

	return probelbl_updated;
}



bool stop = false;

void *RecvAndProcessData(void *arg)
{

	header recvHead;
	char bufferRecv[4000];
	float conv_float[256] = {0.0};
	uint16_t numfieldsconv = 0;

	while(!stop)
	{
		memset(bufferRecv,0,512);
//if()
//{
		if((size = recv(sockClient , &recvHead , sizeof(recvHead) , 0)) < 0)
		{
			//fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
			printf("RECEIVE FAILED\n");
			//return -1;
		}

		if(recvHead.prefix != 0xFFFFFFFF) continue;

		//printf("Processing command ->%d-hex %x--Len %d - hex %x \r\n",recvHead.command,recvHead.command,recvHead.length,recvHead.length);
		recvHead.length = DecodeInt32(recvHead.length);
		recvHead.command = DecodeInt32(recvHead.command);
//}
		//printf("After converting command ->%d-hex %x--Len %d - hex %x \r\n",recvHead.command,recvHead.command,recvHead.length,recvHead.length);
		//BMJO need modify loop to handle small chunks of data and where multiple reads required

		if(recvHead.length > 0)
		{
			if((size = recv(sockClient ,bufferRecv , recvHead.length , 0)) < 0)
			{
				//fprintf(stderr, "recv: %s (%d)\n", strerror(errno), errno);
				printf("recv failed\n");
				//return -1;
			}
		}
		switch(recvHead.command )
		{
			case RESP_BATINFO:
				printf("Battery Info: %s\r\n",bufferRecv);
				stringtofloat(bufferRecv,conv_float,&numfieldsconv);
				battery.percent = (uint8_t)conv_float[2];
				battery.charging_status = (uint8_t)conv_float[1];
				disp_update_device_status();

				break;
			case RESP_MEAS_START:
				//rest_meas_start = true;
				stringtofloat(bufferRecv,conv_float,&numfieldsconv);
				printf("Response Measurement Start: %f\n",conv_float[0]);
				measurement_start();
				UpdateStatusLabel(true ,contraction_flag);
				probelbl_updated = UpdateProbelabel((int)conv_float[0]);
				break;

			case RESP_MEAS_STOP:
				//rest_meas_start = false;
				if(is_start_capture == true)
				{
					is_start_capture = false;
					/*time(&end_time);
					diff_time = difftime(end_time,start_time);
					printf("Difference time: %f\n",diff_time);*/
				}

				if(meausrement_settings.measurement_type > 1)
				{//either MVC or endurance
					store_contraction_smry();
				}
				printf("Measurement Stop: %s\r\n",bufferRecv);
				UpdateStatusLabel(false,contraction_flag);
				//MeasBeep(3);
				//DisablePcSpkr();
				break;

			case RESP_MEAS_RMS:
				//printf("RESP_MEAS_RMS Start\n");
				if(probelbl_updated == false)
				{
					UpdateProbelabel(0);
					//rest_meas_start = true;
					measurement_start();
					UpdateStatusLabel(true ,contraction_flag);
				}

				if(is_start_capture == false)
				{
					is_start_capture = true;
					time(&start_time);
				}

				ProcessRMSData(bufferRecv, false);

				break;
			case RESP_MEAS_RAWDAT:
				printf("RAW Data Recvd Len -> %d \n",recvHead.length );
				elcs = (elctrodes *)bufferRecv;
				elcs->numsamples = DecodeInt32(elcs->numsamples);
				for(int i = 1; i <= (elcs->numsamples * MAXCHANNELS);i++)
				{
					elcs->elec[i-1] = DecodeInt32(elcs->elec[i-1]);
					elcs->elec[i-1]=(int) elcs->elec[i-1]>>8;
					rawdata_mv[i-1] = MilliVoltConv(elcs->elec[i-1]);
					//printf("%d,%f",elcs->elec[i-1],rawdata_mv[i-1]);
					//printf("%f,",rawdata_mv[i-1]);
					//if((i%24 == 0) && (i !=0)) printf("\n");
				}

				filter(rawdata_mv,filtered_output,(elcs->numsamples * MAXCHANNELS));
				/*for(int i = 1; i <= (elcs->numsamples * MAXCHANNELS);i++)
				{
					fprintf(fp_data,"\n%d,%f,%f",elcs->elec[i-1],rawdata_mv[i-1],filtered_output[i-1]);
					if((i%24 == 0) && (i !=0)) fprintf(fp_data,"\n");
				}*/

				break;

			case RESP_MEAS_RMS_SUMMARY:
				if(meausrement_settings.measurement_type == 1)
				{
					printf("RMS Summary: %s \n",bufferRecv);
					stringtofloat(bufferRecv,conv_float,&numfieldsconv);
					getTimestamp(cont_start_time);
					//Storing RMS Summary file vmj
					for(int i = 1;i < (numfieldsconv); i++)
					{
						fprintf(fp_summary,"%f,",conv_float[i]);
						//printf("%f,",conv_float[i]);
					}
					fprintf(fp_summary,"%s,",cont_start_time);
					fprintf(fp_summary,"%s,","Overall");
					/*for(int i = 1;i < (numfieldsconv); i++)
					{
						fprintf(fp_data,"%f,",conv_float);
						//printf("%f,",conv_float[i]);
					}
					 */

				}
				else if(meausrement_settings.measurement_type > 1)
				{
					//scale_fun(array_total_rms_value,&scale,array_total_rms_value,24);
					mean_fun(array_total_rms_value,24,&array_total_rms_value[24]);
					/*printf("contraction total summary in receive fun\n");
					for(int i = 0; i<= 24 ;i++)
					{
						printf("%f,",array_total_rms_value[i]);
					}
					printf("\n");*/
				}

				break;
			case RESP_MEAS_CON_START:
				printf("CON_START \n");
				contraction_flag = true;
				rms_stop_command_once = true;
				getTimestamp(cont_start_time);
//				veeresh tseting purpose
				time(&rawtime);
				timeinfo =localtime ( &rawtime );
				start_time_con.min = timeinfo->tm_min;
				start_time_con.sec = timeinfo->tm_sec;

				cont_cycles_count++;
				//printf("\nCont_cycles_count:%d\n",cont_cycles_count);
				//MeasBeep(2);

				break;

			case RESP_MEAS_CON_STOP:
			//	printf("CON_STOP :%s \n",bufferRecv);

				break;

			case RESP_MEAS_CON_SUMMARY:
				//printf("CON_Summary: %s \n",bufferRecv);

				break;

			case -1190985728:
				gtk_label_set_markup (GTK_LABEL(connlbl), disconnected_attr);
				gtk_label_set_markup (GTK_LABEL(probe_vallbl), disconnected_attr);
				is_device_connected = false;
				//probelbl_updated = false;
				gtk_widget_set_sensitive(reconnect_button,true);
				gtk_widget_set_sensitive(start_button,false);
				gtk_widget_set_sensitive(pause_button,false);
				gtk_widget_set_sensitive(save_button,false);
				break;

			case -1224540160:
				if(probelbl_updated == false)
				{
					UpdateProbelabel(-34);
					gtk_label_set_markup (GTK_LABEL(connlbl), disconnected_attr);
					is_device_connected = false;

					gtk_widget_set_sensitive(reconnect_button,true);
					gtk_widget_set_sensitive(start_button,false);
					gtk_widget_set_sensitive(pause_button,false);
					gtk_widget_set_sensitive(save_button,false);
				}
				break;
			default :
				printf("Unknown command ->%d\r\n",recvHead.command);
				break;
		}
		//printf("Resp : %s\n", &bufferRecv[12]);
	}


}
void contraction_process(void)
{
	count_before_con_strt++;
	if(contraction_flag)
	{
		RMS_sample_count++;
		contraction_data_calculation();
	}
	else
	{
		for(int i = 0;i<23;i++)
		{
			if(rms.rms_val[i] < minval && rms.rms_val[i] != -1)
			{
				minval = rms.rms_val[i];
				timeTorest  = count_before_con_strt;
			}
		}

	}
}

void store_contraction_smry()
{
	//int cont_count = 0;
	cont_count = cont_cycles_count ;
	cont_cycles_count = 0;
	if(rms_stop_command_once )
	{
		printf("contraction summary\n");
		for(int i = 0;i<24;i++)
		{
			array_cont_cmplt_smry[i] /= cont_count;
		}
		mean_fun(array_cont_cmplt_smry,24,&array_cont_cmplt_smry[24]);
		for(int i = 0;i <= 24; i++)
		{
			//printf("%f, ",array_cont_cmplt_smry[i]);
			fprintf(fp_summary,"%f,",array_cont_cmplt_smry[i]);
		}
		//all 4 values are just to fill the blank place
 		fprintf(fp_summary,"%d,",0);
		fprintf(fp_summary,"%d,",0);
		fprintf(fp_summary,"%d,",0);
		fprintf(fp_summary,"%d,",0);
		fprintf(fp_summary,"%d,",0);

		fprintf(fp_summary,"%d,",0);

		fprintf(fp_summary,"%s,\n","Overall");

		//printf("\n");
		rms_stop_command_once = false;
	}
	cont_count = 0;
}

void measurement_start(void)
{
	for(int i = 0;i < 25 ;i++)
	{
		array_total_rms_value[i] = 0;
		array_cont_rms_value[i] = 0;
		array_cont_cmplt_smry [i] = 0;
	}
	cont_count = 0;
	peakval = 0;
	timeTopeak = 0;
	timeTorest = 0;
	cont_cycles_count = 0;
}


void calculate_running_avg(void)
{
	char disp_sum_buf[25];
	float array_total_rms = 0,temp,runningAvg=0;
	for(int i =0 ; i < 24 ; i++)
	{
		array_total_rms_value[i]+= rms.rms_val[i];
	}
	if(totSampleRecvd % 50 == 0)
	{
		for(int i =0 ; i < 24 ; i++)
		{
			temp = array_total_rms_value[i]/totSampleRecvd;
			array_total_rms+=temp;
		}
		runningAvg = array_total_rms/24;
		sprintf(disp_sum_buf,"  Average     : %0.2f",runningAvg);
		gtk_label_set_text(GTK_LABEL(avglbl),disp_sum_buf);
	}
}

void contraction_data_calculation()
{
	scale =(float)1/(float)(meausrement_settings.measure_duration * 10);

for(int i =0 ; i < 24 ; i++)
	{
		array_total_rms_value[i]+= rms.rms_val[i];
	}
	if((RMS_sample_count) < (meausrement_settings.active_time*10))//10total number of sample per sec
	{
		for(int i = 0;i < 24;i++)
		{
			array_cont_rms_value[i] += rms.rms_val[i];
			if(rms.rms_val[i] > peakval)
			{
				peakval = rms.rms_val[i];
				timeTopeak = RMS_sample_count;
			}
		}
	}
	else
	{
		getTimestamp(cont_stop_time);
		cont_count++;
		//printf("CONT STOP \n");
		for(int i =0 ;i < 24;i++)
		{
			array_cont_rms_value[i] = array_cont_rms_value[i] / RMS_sample_count;
		}
		mean_fun(array_cont_rms_value,24,&array_cont_rms_value[24]);

		//changed by veeresh verify with bmjo
		/*printf("avg of contn start \n");
		float cont_rms_24 =0;
		for(int i = 0;i < 24 ; i++)
		{
			cont_rms_24 +=array_cont_rms_value[i];
		}
		printf("avg of contn end  \n");

		cont_rms_24/=24;
		array_cont_rms_value[24]=cont_rms_24;*/

//added by veeresh j to get the complete summary
		for (int i = 0;i < 24;i++ )
		{
			array_cont_cmplt_smry[i] = array_cont_rms_value[i];
		}

		/*printf("rest val = %f\t time to rest = %d\n",minval,timeTorest);
		for(int i = 0;i <= 24 ;i++)
		{
			printf(" %f ,",array_cont_rms_value[i]);
		}
		printf("\n");
		printf("peak val = %f\t time to peak=%d\n",peakval,timeTopeak);
		*/

		for(int i = 0;i <= 24; i++)
		{
			fprintf(fp_summary,"%f,",array_cont_rms_value[i]);
		}

		//printf("file opern end \n");
		fprintf(fp_summary,"%f,",peakval);
		fprintf(fp_summary,"%d,",timeTopeak);
		fprintf(fp_summary,"%f,",minval);
		fprintf(fp_summary,"%d,",timeTorest);

		/* time and date */
		fprintf(fp_summary,"%s,",cont_start_time);
		fprintf(fp_summary,"%s,",cont_stop_time);

		//fprintf(fp_summary,"%d:",start_time_con.min);
		//fprintf(fp_summary,"%d,",start_time_con.sec);
		fprintf(fp_summary,"%s","Contraction");
		fprintf(fp_summary,"%d,\n",cont_count-1);//to start count from zero

		char disp_sum_buf[30];

		sprintf(disp_sum_buf,"   Average    : %0.2f",array_cont_rms_value[24]);
		gtk_label_set_text(GTK_LABEL(avglbl),disp_sum_buf);

		sprintf(disp_sum_buf,"   Peak            : %0.2f",peakval);
		gtk_label_set_text(GTK_LABEL(peaklbl),disp_sum_buf);

		sprintf(disp_sum_buf,"Onset          : %d",timeTopeak);
		gtk_label_set_text(GTK_LABEL(onsetlbl),disp_sum_buf);

		sprintf(disp_sum_buf,"Offset         : %d",timeTorest);
		gtk_label_set_text(GTK_LABEL(offsetlbl),disp_sum_buf);

		minval = peakval;
		contraction_flag = false;
		peakval = 0 ;
		timeTopeak = 0;
		timeTorest = 0;
		RMS_sample_count = 0;
		count_before_con_strt = 0;
		//UpdateStatusLabel(true,contraction_flag);
		printf("Contraction Stop\n");
		//MeasBeep(1);
		//bmjo todo - update display with peak ,average etc
	}
}


void scale_fun(float ary[],float* scale_val,float dest[],int blk_size)
{
	for(int i = 0;i < blk_size;i++)
	{
		dest[i] = ary[i]*(*scale_val);
	}
}


void mean_fun(float ary[],int size,float*dest)
{
	float result = 0 ;

	for(int i = 0;i<size;i++)
	{
		result += ary[i];
	}

	result /=24;

	*dest = result;
}

void displayXYScaleInit(int xWindowDurationSec, int yMilliVol)
{
	if(yMilliVol > MAX_YVAL_MV)
		xy_axis_range.y_max_range= MAX_YVAL_MV;
	else
		xy_axis_range.y_max_range = yMilliVol;

	measure_diplay_window_samples = xWindowDurationSec*10;

	if(xy_axis_range.x_max_range > MAX_SAMPLES)
		xy_axis_range.x_max_range = MAX_SAMPLES;
	else
		xy_axis_range.x_max_range = measure_diplay_window_samples;
}
void db_init()
{
	MYSQL_RES *res;

	char *server = "127.0.0.1";// use this if accessing from local s/m, else "Localhost" if running code frm server. Date: 22-07-2020
	char *user = "root";
	char *password = "";// password is set in this example
	char *database = "mapledb";
	 conn = mysql_init(NULL);

	  /* Connect to database */
	  if (!mysql_real_connect(conn, server,user, password, database, 0, NULL, 0))
	  {
		printf("Failed to connect MySQL Server %s. Error: %s\n", server, mysql_error(conn));
		//return 0;
	  }

	printf("Measurement parameters\n");
	if (mysql_query(conn, "SELECT tmp.PrescriptionUUID,mpm.Measurement_Type,mp.Measurement_Duration,mp.Contraction_Count,mp.ActiveContract,mp.RestContract,mp.EMG_Sensitivity,mp.EMG_Timespan,mpm.Instrument_Location FROM TempMeasurementSettings tmp, MeasurementPrescription mp,MeasurementPrescriptionMaster mpm WHERE (tmp.PrescriptionUUID = mp.PrescriptionUUID) AND (tmp.PrescriptionUUID = mpm.PrescriptionId) ORDER BY tmp.id DESC LIMIT 1 ;"))
	{
		printf("Failed to execute query. Error: %s\n", mysql_error(conn));
		//return 0;
	}


	res = mysql_store_result(conn);

	if (res == NULL)
	{
		printf("Error\n");
	  //finish_with_error(conn);
		//return 0;
	}

	row = mysql_fetch_row(res);

	printf("%s\n", row[0]);


	char extension[12] = ".txt";


	meausrement_settings.active_time = atoi(row[4]);
	meausrement_settings.rest_time = atoi(row[5]);
	meausrement_settings.contraction_count = atoi(row[3]);

	meausrement_settings.emg_sensitivity = atoi(row[6]);
	meausrement_settings.emg_timespan = atoi(row[7]);
	meausrement_settings.rest_timecount = meausrement_settings.rest_time*10;
	meausrement_settings.activetime_count = meausrement_settings.active_time*10;
	//bmjo todo :- remove below line
	//meausrement_settings.measure_duration =180;
	//Read from SQL


	if((strcmp(row[1] ,"Rest")) == 0)
	{
		meausrement_settings.measurement_type = 1;
		meausrement_settings.measure_duration = (atoi(row[2]) * 60);

	}
	else if((strcmp(row[1] ,"MVC")) == 0)
	{
		meausrement_settings.measurement_type = 2;
		meausrement_settings.measure_duration = meausrement_settings.contraction_count * (meausrement_settings.active_time + meausrement_settings.rest_time);
	}
	else if((strcmp(row[1] ,"Endurance")) == 0)
	{
		meausrement_settings.measurement_type = 3;
		meausrement_settings.measure_duration = meausrement_settings.contraction_count * (meausrement_settings.active_time + meausrement_settings.rest_time);
	}
	else
	{
		meausrement_settings.measurement_type = 0;
		meausrement_settings.measure_duration = (atoi(row[2]) * 60);
	}


	//Modification for testing REST tpe
	//meausrement_settings.measure_duration = 20;
	//meausrement_settings.measurement_type = 1;


	//Modification for testing MVC
	//meausrement_settings.measure_duration = 30;
//	meausrement_settings.measurement_type = 2;
//	meausrement_settings.active_time = 2;
//	meausrement_settings.rest_time = 3;
//	meausrement_settings.contraction_count = 5;
//	meausrement_settings.measure_duration = meausrement_settings.contraction_count * (meausrement_settings.active_time + meausrement_settings.rest_time);

	//if((meausrement_settings.measurement_type == 2)||(meausrement_settings.measurement_type == 3))
	{
		strcat(filename_summary,row[0]);
		strcat(filename_summary,"_summary");
		strcat(filename_summary,extension);
		fp_summary = fopen(filename_summary, "w+");
	}

	strcat(data_file,row[0]);
	strcat(data_file,"_data");
	strcat(data_file,extension);
	fp_data = fopen(data_file, "w+");

	printf("File_Name MVC: %s\n",data_file);
	printf("filename_summary: %s\n",filename_summary);

	printf("MT: %d\n",meausrement_settings.measurement_type);

	//Todo Rashmi :- read diaplsy time slot and yaxis from measurement settings
	displayXYScaleInit(meausrement_settings.emg_timespan, meausrement_settings.emg_sensitivity);
	timeRemaining = meausrement_settings.measure_duration;


}
void db_initDummy(void)
{


	meausrement_settings.active_time = 100;
	meausrement_settings.rest_time = 10;
	meausrement_settings.contraction_count =10;

	meausrement_settings.emg_sensitivity = 10;
	meausrement_settings.emg_timespan = 10;
	meausrement_settings.rest_timecount = meausrement_settings.rest_time*10;
	meausrement_settings.activetime_count = meausrement_settings.active_time*10;
	//bmjo todo :- remove below line
	//meausrement_settings.measure_duration =180;
	//Read from SQL


		meausrement_settings.measurement_type = 1;
		meausrement_settings.measure_duration = (atoi(row[2]) * 60);



	//Modification for testing REST tpe
	//meausrement_settings.measure_duration = 20;
	//meausrement_settings.measurement_type = 1;


	//Modification for testing MVC
	//meausrement_settings.measure_duration = 30;
//	meausrement_settings.measurement_type = 2;
//	meausrement_settings.active_time = 2;
//	meausrement_settings.rest_time = 3;
//	meausrement_settings.contraction_count = 5;
//	meausrement_settings.measure_duration = meausrement_settings.contraction_count * (meausrement_settings.active_time + meausrement_settings.rest_time);

	//if((meausrement_settings.measurement_type == 2)||(meausrement_settings.measurement_type == 3))
	{
		strcat(filename_summary,"testbard");
		strcat(filename_summary,"_summary");
		strcat(filename_summary,"csv");
		fp_summary = fopen(filename_summary, "w+");
	}

	strcat(data_file,"Testbrd");
	strcat(data_file,"_data");
	strcat(data_file,"csv");
	fp_data = fopen(data_file, "w+");

	printf("File_Name MVC: %s\n",data_file);
	printf("filename_summary: %s\n",filename_summary);

	printf("MT: %d\n",meausrement_settings.measurement_type);

	//Todo Rashmi :- read diaplsy time slot and yaxis from measurement settings
	displayXYScaleInit(meausrement_settings.emg_timespan, meausrement_settings.emg_sensitivity);
	timeRemaining = meausrement_settings.measure_duration;


}


int sampleNumber = 0;
float delta = 3.14/10;
double rnadScale = (float)RAND_MAX/(float)400;

gboolean waveGen()
{

	int nRandonNumber = rand()%((200+1)-2) + 100;
	float curVal = sampleNumber%10 *2+nRandonNumber;
	for(int i=0;i<24;++i)
	{
		rms.rms_val[i] = curVal;

	}
	rms.avg_rms=curVal*2;
	ProcessRMSData(NULL, true);
/*	load_data_to_display(&rms);
	timer_callback(&cb);

	++fillPointerHigh;
	++fillPointerLow;

	//commented by vj
	//printf("Fill pointer in fill : %d\n",fillPointerLow);

	if(fillPointerLow >= xy_axis_range.x_max_range)
	{
		fillPointerLow = 0;
		fillPointerHigh = xy_axis_range.x_max_range;
	}
	++sampleNumber;*/
return TRUE;

}
void testWaveGenFunc()
{
	 char option;
	 srand(time(NULL));

	 g_timeout_add(100, waveGen, NULL);

	 gtk_widget_show_all(window_main);
	 gtk_main();

	option = getchar();
	if(option == 'q')
	{
		stop=true; //request thread to stop
		printf("Leaving app");

	}
}



char opt;
void *BeepFunc(void *arg)
{

    if(opt==1)
    {
      if(system("play /home/ubuntu/MeasFiles/audio/beep-02.wav")==-1)
      {
    	  printf("failed to run");  //prints !!!Hello World!!!
      }
    }
	if(system("beep -f 3000 -l 100")==-1)
		{
			printf("failed to run");  //prints !!!Hello World!!!
		}

    if(opt==2)
    {
    	if(system("play /home/ubuntu/MeasFiles/audio/beep-01a.wav")==-1)
		  {
			  printf("failed to run");  //prints !!!Hello World!!!
		  }
     }
    if(opt==3)
    {
    	if(system("play /home/ubuntu/MeasFiles/audio/beep-07.wav")==-1)
  		{
  			printf("failed to run");  //prints !!!Hello World!!!
  		}
     }
}

void MeasBeep(char f)
{
	pthread_t thread_id;
	opt = f;

	pthread_create(&thread_id, NULL, BeepFunc, NULL);
}

void EnablePcSpkr()
{
	if(system("sudo modprobe pcspkr")==-1)
	{
		printf("failed to run");  //prints !!!Hello World!!!
	}

}

void DisablePcSpkr()
{
	if(system("sudo modprobe --remove pcspkr")==-1)
	{
		printf("failed to run");  //prints !!!Hello World!!!
	}

}
char commandBuf[4096];
int read_error;
bool lookForPrefix()
{
	char readBuf[100];
	while(1){
		memset(readBuf,0,100);
		int num_bytes = read(serial_port, readBuf, 1);
		if(num_bytes==-1){read_error = SERIAL_PORT_READ_ERROR; return false;}
		if(readBuf[0]=='\n') return true;
		//printf("%c",readBuf[0]);
	}

}

bool readCommandChar(){
	char read_buf[256];
	int cmdBufPtr = 0;
	char * loc;
	while(1)
	{
		memset(&read_buf, '\0', sizeof(read_buf));
		int num_bytes = read(serial_port, &read_buf, 1);
		if(num_bytes<=0){read_error = SERIAL_PORT_READ_ERROR; return false;}
		commandBuf[cmdBufPtr] = read_buf[0];
		if((read_buf[0]=='\n')&&(cmdBufPtr>10))
		{
			printf("%s",commandBuf);
			return true;
		}
		++cmdBufPtr;
		if(cmdBufPtr >= COMMAND_BUF_SIZE){ read_error = SERIAL_PORT_BUF_OVERRUN; return false;}
	}
}

bool readCommand(){
	char read_buf[256];
	int cmdBufPtr=0;
	char * loc;
	while(1){
		memset(&read_buf, '\0', sizeof(read_buf));

		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
		if(num_bytes>0){

		if((loc=strchr(read_buf,'\n'))!=NULL){
			num_bytes = loc-read_buf+1;
			if((cmdBufPtr+num_bytes)>=COMMAND_BUF_SIZE){ read_error = SERIAL_PORT_BUF_OVERRUN; return false;}
			memcpy(&commandBuf[cmdBufPtr],loc,num_bytes);
			return true;
		}else{
			if((cmdBufPtr+num_bytes)>=COMMAND_BUF_SIZE){ read_error = SERIAL_PORT_BUF_OVERRUN; return false;}
			memcpy(&commandBuf[cmdBufPtr],read_buf,num_bytes);
		}
		cmdBufPtr+=num_bytes;
		}else {
			read_error = SERIAL_PORT_READ_ERROR;
			return false;
		}
	}
}
void  processSerialData(commandBuf)
{
	// printf(" Received message: %s\n", commandBuf);

}
//Start this as a thread after initaliing serial port and UI
void *RecvAndProcessSerialData(void *arg)
{
	set_blocking(serial_port,1);

	while(true){
		if(!lookForPrefix()) return false;
		memset(commandBuf,0,COMMAND_BUF_SIZE);
		if(readCommandChar()) {

		  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
		  // print it to the screen like this!)

		  processSerialData(commandBuf);
		  SerialDataParsing(commandBuf,false);
		}else {
			if(read_error== SERIAL_PORT_READ_ERROR) return false;
		}
	}
}
bool DataRx()
{
	set_blocking(serial_port,1);

	while(true){
		if(!lookForPrefix()) return false;
		memset(commandBuf,0,COMMAND_BUF_SIZE);
		if(readCommandChar()) {
		  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
		  // print it to the screen like this!)
		  printf(" Received message: %s\n", commandBuf);
		}else {
			if(read_error== SERIAL_PORT_READ_ERROR) return false;
		}
	}
}
void MapleHomeDeviceDataRx()
{
	pthread_t thread_id1;
	//DataRx();
	pthread_create(&thread_id1,NULL,DataRx,NULL);

	//pthread_create(&thread_id1,NULL,serialPortInit,NULL);

}

void TxCommand()
{
//	unsigned char msg[] = "Hello,world!\n";


	char cmd[] = "\r51##110000000000000000000000,900,60,2,1,0,10,1,3,3$$";

	//char cmd[] = "\r70#0$$";

	//int size = sizeof(cmd);
	//write(serial_port, (char*)cmd, strlen(cmd));

	usleep ((strlen(cmd) + 25) * 100);//wait till the transmission to complete


	char cmd1[] = "\r50#110000000000000000000000,900,60,2,1,0,10,1,3,3$";
	write(serial_port, (char*)cmd1, strlen(cmd1));

	usleep ((strlen(cmd1) + 25) * 100);//wait till the transmission to complete

}
void TxCommand_stop()
{

	char cmd[] = "\r51##110000000000000000000000,900,60,2,1,0,10,1,3,3$$";

	//char cmd[] = "\r70#0$$";

	//int size = sizeof(cmd);
	write(serial_port, (char*)cmd, strlen(cmd));

	usleep ((strlen(cmd) + 25) * 100);//wait till the transmission to complete

}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	printf("Error reading: %s", strerror(errno));
             //   error_message ("error %d from tggetattr", errno);
                return;
        }
       // tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VMIN]  = 4096;
        tty.c_cc[VTIME] = 10;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	printf("Error reading: %s", strerror(errno));
        	//error_message ("error %d setting term attributes", errno);
}
void delay(int milli_seconds)
{
    // Converting time into milli_seconds
    //int milli_seconds = 1000 * number_of_seconds;

    // Storing start time
    clock_t start_time = clock();

    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

int main(int argc, char *argv[])
{
	debugSetup = true;
	serialPortInit();
	TxCommand();
	//MapleHomeDeviceDataRx();

/*	while(1)
	{
		//serialPortInit();
		//close(serial_port);
		DataRx();
		//TxCommand();
		//TxCommand_dummy();
		//TxCommand();
		//delay(1000);
		//for(int i = 0;i<10000;i++);
		//TxCommand();
		//DataRx();
		//close(serial_port);
		//MapleHomeDeviceDataRx();
	}puts("!!!Hello World!!!");  //prints !!!Hello World!!!*/
	pthread_t thread_id;
	////MeasBeep(1);




	//deafult initialization for safety
	//displayXYScaleInit(60,200);
	//testWaveGenFunc();

	//displayXYScaleInit(meausrement_settings.emg_timespan,meausrement_settings.emg_sensitivity);
if(debugSetup == false)
{
	gtk_init(&argc, &argv);

	db_init();
	window_init();
	chart_init();
	RequestServDisc(); //need to parse the response and put that in global structure
	if(ConnectToDev())
	{
		pthread_create(&thread_id, NULL, RecvAndProcessData, NULL);
		RequestInfoBattery(); //need to parse the response and place it in struct named battoryinfo

		//RequestMeasStart_RMS();

		//RequestMeasStart_Raw();

	}

}
else if(debugSetup == true)
{
	gtk_init(&argc, &argv);
	//db_init();
	displayXYScaleInit(30,200);

	window_init();
	chart_init();
	pthread_create(&thread_id, NULL, RecvAndProcessSerialData, NULL);

}

	displayPointer = 0; //display pointer
	//fillPointer = DISP_NUM_SAMPLES; //fill pointer point to the end of display window so that grapgh can scroll in
//	while(1)
//	{

//	}
	gtk_widget_show_all(window_main);
	gtk_main();

	//gtk_widget_show_all(window);
	//gtk_main();

	close(serial_port);//close serial port communication
	close(sockClient);
	fclose(fp_data);
	mysql_close(conn);

	return EXIT_SUCCESS;
}
