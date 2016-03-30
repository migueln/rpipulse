#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#include <time.h>
#include <math.h>

#include <pigpio.h>

typedef enum { false, true } bool;

/*
2014-08-20

gcc -o phot phot.c -lpigpio -lpthread -lm -lrt 
$ sudo ./phot  4 7 8
*/

#define MAX_GPIOS 32

#define OPT_R_MIN 1
#define OPT_R_MAX 1200
#define OPT_R_DEF 5

#define OPT_S_MIN 1
#define OPT_S_MAX 10
#define OPT_S_DEF 5

#define OPT_T_MIN 1
#define OPT_T_MAX 600
#define OPT_T_DEF 1

#define CAL_MAG_1Hz 2200
#define SLOWMODE_TRIGH 1200 // Freq needed to switch to fast mode
#define SLOWMODE_TRIGL 800  // Freq needed to switch to slow mode


struct timespec gettime_now;
struct timespec gettime_prev[MAX_GPIOS];
static volatile long time_difference[MAX_GPIOS]; // usec
static uint32_t g_mask;
char timebuffer[26];

static volatile int g_pulse_count[MAX_GPIOS];
static volatile int g_reset_counts[MAX_GPIOS];
static uint32_t g_mask;

static int g_num_gpios;
static int g_gpio[MAX_GPIOS];

static int g_opt_T = OPT_T_DEF;
static int g_opt_r = OPT_R_DEF;
static int g_opt_s = OPT_S_DEF;
static int g_opt_t = 0;

static int nwanted[MAX_GPIOS];
static int nsbmag[MAX_GPIOS];
static long period[MAX_GPIOS];
static long freque[MAX_GPIOS];

bool slowmode[MAX_GPIOS];   // Keep track of reading mode for each sensor
bool haschanged[MAX_GPIOS]; // Has the reading mode changed?
bool isready[MAX_GPIOS];    // Are the measurements already stable?


void usage()
{
   fprintf
   (stderr,
      "\n" \
      "Usage: sudo ./phot gpio ... [OPTION] ...\n" \
      "   -r value, sets refresh period in deciseconds, %d-%d, default %d\n" \
      "   -s value, sets sampling rate in micros,       %d-%d, default %d\n" \
      "   -t value, set pulse detection timeout (sec),  %d-%d, default %d\n" \
      "\nEXAMPLE\n" \
      "sudo ./phot 4 7 -r2 -s2\n" \
      "Monitor gpios 4 and 7.  Refresh every 0.2 seconds.  Sample rate 2 micros.\n" \
      "\n",
      OPT_R_MIN, OPT_R_MAX, OPT_R_DEF,
      OPT_S_MIN, OPT_S_MAX, OPT_S_DEF,
      OPT_T_MIN, OPT_T_MAX, OPT_T_DEF
   );
}

void fatal(int show_usage, char *fmt, ...)
{
   char buf[128];
   va_list ap;

   va_start(ap, fmt);
   vsnprintf(buf, sizeof(buf), fmt, ap);
   va_end(ap);

   fprintf(stderr, "%s\n", buf);

   if (show_usage) usage();

   fflush(stderr);

   exit(EXIT_FAILURE);
}

static int initOpts(int argc, char *argv[])
{
   int i, opt;

   while ((opt = getopt(argc, argv, "t:r:s:")) != -1)
   {
      i = -1;

      switch (opt)
      {
         case 't':
            i = atoi(optarg);
            if ((i >= OPT_T_MIN) && (i <= OPT_T_MAX))
               g_opt_T = i;
            else fatal(1, "invalid -r option (%d)", i);
            break;
         case 'r':
            i = atoi(optarg);
            if ((i >= OPT_R_MIN) && (i <= OPT_R_MAX))
               g_opt_r = i;
            else fatal(1, "invalid -r option (%d)", i);
            break;
         case 's':
            i = atoi(optarg);
            if ((i >= OPT_S_MIN) && (i <= OPT_S_MAX))
               g_opt_s = i;
            else fatal(1, "invalid -s option (%d)", i);
            break;

        default: /* '?' */
           usage();
           exit(-1);
        }
    }
   return optind;
}

long int timediff(struct timespec start, struct timespec end){
	long nsec, sec;
	if (end.tv_nsec-start.tv_nsec<0) {
		sec  = end.tv_sec-start.tv_sec-1;
		nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	}
	else {
		sec  = end.tv_sec-start.tv_sec;
		nsec = end.tv_nsec-start.tv_nsec;
	}
	return sec*1e6 + nsec/1e3;
}

void update_gettime(int gpio){
	gettime_prev[gpio].tv_nsec = gettime_now.tv_nsec;
	gettime_prev[gpio].tv_sec  = gettime_now.tv_sec;
}

void measure_times(int gpio){
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	time_difference[gpio] = timediff(gettime_prev[gpio],gettime_now);
	update_gettime(gpio);
}

void edges(int gpio, int level, uint32_t tick){
   if (g_reset_counts[gpio]==1) {
      g_pulse_count[gpio]  = 0;
      g_reset_counts[gpio] = 0;
   }
   /* only record low to high edges */
   if (level == 1) g_pulse_count[gpio]++;
}

void times(int gpio, int level, uint32_t tick){
	if (g_reset_counts[gpio]==1) {
		g_pulse_count[gpio]  = 0;
		g_reset_counts[gpio] = 0;
	}
	// only time low to high edges
	if (level == 1){
		g_pulse_count[gpio]++;
		if (g_pulse_count[gpio] == nwanted[gpio]){
			measure_times(gpio);
			isready[gpio]        = 1;
			g_reset_counts[gpio] = 1;
		}
	}
}

void reset_ready_sensors(){
	int i;
	for (i=0; i<g_num_gpios; i++){
		if (isready[g_gpio[i]]==1){
			g_reset_counts[g_gpio[i]]=1;
			isready[g_gpio[i]]=0;
			measure_times(g_gpio[i]);
		}
	}
}

void get_current_datetime(){
	time_t timer;
	struct tm* tm_info;
	time(&timer);
	tm_info = localtime(&timer);
	strftime(timebuffer, 26, "%Y:%m:%d %H:%M:%S", tm_info);
}


void get_period_and_freq(){
	int i, g;
   for (i=0; i<g_num_gpios; i++){
		g = g_gpio[i];
		if (slowmode[i]){
			period[i] = time_difference[g]/nwanted[g];
			freque[i] = 1000000./(1e-3+period[i]);
		}
		else{
			freque[i] = g_pulse_count[g]*10/g_opt_r;
			period[i] = 1000000./(1e-3+freque[i]);
		}
		nsbmag[i] = (int) (CAL_MAG_1Hz - 250*log10(1e-3 + freque[i])+0.5);
	}
	//reset_ready_sensors();
}

void optimize_detection_mode(){
	int i, g;
   for (i=0; i<g_num_gpios; i++){
		haschanged[i] = 0;
		if (slowmode[i]==1 && freque[i]>=SLOWMODE_TRIGH){
			printf("# Switching to fast mode\n");
			haschanged[i]=1;
			slowmode[i]=0;
			g_pulse_count[g_gpio[i]] = 0;
			gpioSetAlertFunc(g_gpio[i], edges);
		}
		else if (slowmode[i]==0 && freque[i]<SLOWMODE_TRIGL){
			printf("# Switching to slow mode\n");
			haschanged[i]=1;
			slowmode[i]=1;
			g_pulse_count[g_gpio[i]] = 0;
			gpioSetAlertFunc(g_gpio[i], times);
		}
		nwanted[g_gpio[i]] = freque[i]/100 + 2;
	}
}

int count_ready_sensors(){
	int i;
	int res = 0;
	for (i=0; i<g_num_gpios; i++) if (isready[g_gpio[i]]==1) res++;
	return res;
}

int count_changedmode_sensors(){
	int i;
	int res = 0;
	for (i=0; i<g_num_gpios; i++) if (haschanged[i]==1) res++;
	return res;
}

int setup(int argc, char *argv[])
{
   int i, rest, g, wave_id;
   gpioPulse_t pulse[2];
   /* command line parameters */
   rest = initOpts(argc, argv);
   /* get the gpios to monitor */
   g_num_gpios = 0;
   for (i=rest; i<argc; i++)
   {
      g = atoi(argv[i]);
      if ((g>=0) && (g<32))
      {
         g_gpio[g_num_gpios++] = g;
         g_mask |= (1<<g);
      }
      else fatal(1, "%d is not a valid g_gpio number\n", g);
   }
   if (!g_num_gpios) fatal(1, "At least one gpio must be specified");

   printf("Monitoring gpios");
   for (i=0; i<g_num_gpios; i++) printf(" %d", g_gpio[i]);
   printf("\nSample rate %d micros, refresh rate %d deciseconds\n",
      g_opt_s, g_opt_r);

   gpioCfgClock(g_opt_s, 1, 1);
   if (gpioInitialise()<0) return 1;
	for (i=0; i<g_num_gpios; i++){
		gpioSetMode(g_gpio[i], PI_INPUT);
		gpioSetAlertFunc(g_gpio[i], edges);
	   g_reset_counts[g_gpio[i]] = 1;
	   measure_times(g_gpio[i]);
	}
	gpioDelay(1000000);
	get_period_and_freq();
	optimize_detection_mode();
   //reset_ready_sensors();
}

void print_results(){
	int i;
        get_current_datetime();
	for (i=0; i<g_num_gpios; i++){
		printf("> %19s, GPIO %d, %7ld Hz, %7ld us, %3d.%02d mag\n",
				timebuffer, g_gpio[i], freque[i], period[i], 
				nsbmag[i]/100, nsbmag[i]%100);
	}
}


void measure_fluxes(bool opt)
{
	int i;
	struct timespec gettime_temp;
	
	// Set fast mode sensors as not ready
	for (i=0; i<g_num_gpios; i++){
		if (slowmode[g_gpio[i]]==0)
	   	g_reset_counts[g_gpio[i]]=1; // this will reset the counts
	}
	reset_ready_sensors();	
	gpioDelay(g_opt_r * 100000);
	// Mark fast mode sensors as ready
	for (i=0; i<g_num_gpios; i++)
		if (slowmode[g_gpio[i]]==0) isready[g_gpio[i]]=1;

	// Wait until the rest of the sensors are ready
	while(count_ready_sensors()<g_num_gpios) {};
	get_period_and_freq();

	if (opt==true)
		if (count_changedmode_sensors()==0) print_results();
	else
		print_results();
   
   optimize_detection_mode();
   reset_ready_sensors();
}

int main(int argc, char *argv[])
{
   setup(argc,argv);
   while (1)
   {
		measure_fluxes(1);
   }
   gpioTerminate();
}

