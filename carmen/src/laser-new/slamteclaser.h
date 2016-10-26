#ifndef SLAMTECLASER_H
#define SLAMTECLASER_H

/* needed for new carmen_inline def for gcc >= 4.3 */
#include <carmen/carmen.h>

//#define HOKUYO_ALWAYS_IN_SCIP20

#define RPLIDAR_BUFSIZE 8192
#define RPLIDAR_ANGULAR_STEP (M_PI/180)
#define RPLIDAR_MAX_BEAMS 360 //1
#define RPLIDAR_DETECTION_RANGE_START 0
#define RPLIDAR_DETECTION_RANGE_END 6
#define RPLIDAR_MAX_RANGE 6.0
#define RPLIDAR_ACCURACY .0001
#define RPLIDAR_FOV (360*M_PI/180.0)
#define RPLIDAR_START_ANGLE 0 //(-UTM_FOV/2.0)

typedef struct SlamtecRangeReading{
  int timestamp;
  int status;
  int n_ranges;
  unsigned short ranges[RPLIDAR_MAX_BEAMS];  //use the UTM size, which wastes a bit of space for URG, but whatever...
  unsigned short startStep, endStep, clusterCount;
} SlamtecRangeReading;

#if 0
typedef struct HokuyoLaser{
  int fd;
  int isProtocol2;
  int isContinuous;
  int isInitialized;
} HokuyoLaser;

// opens the hokuyoLaser, returns <=0 on failure
int hokuyo_open(HokuyoLaser* hokuyoLaser, const char* filename);

// initializes the hokuyoLaser and sets it to the new scip2.0 protocol
// returns <=0 on failure
int hokuyo_init(HokuyoLaser* hokuyoLaser);

// reads a packet into the buffer
unsigned int hokuyo_readPacket(HokuyoLaser* hokuyoLaser, char* buf, int bufsize, int faliures);

// starts the continuous mode
int hokuyo_startContinuous(HokuyoLaser* hokuyoLaser, int startStep, int endStep, int clusterCount,int scanInterval);

// starts the continuous mode
int hokuyo_stopContinuous(HokuyoLaser* hokuyoLaser);
int hokuyo_reset(HokuyoLaser* hokuyoLaser);
int hokuyo_close(HokuyoLaser* hokuyoLaser);
void hokuyo_parseReading(HokuyoRangeReading* r, char* buffer,carmen_laser_laser_type_t laser_type);
#endif
#endif

