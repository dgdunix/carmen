#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <carmen/carmen.h>
#include "carmen_slamtec.h"
#include "slamteclaser.h"
#include "rplidar.h"

#ifdef __APPLE__
#include <limits.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX
#else
#include <values.h>
#endif

#include "laser_messages.h"

#define UNUSED(x) (void)(x)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif
#define DEBUG 0
using namespace rp::standalone::rplidar;

RPlidarDriver *drv = NULL;
u_result capture_and_display(RPlidarDriver * drv)
{
    u_result ans;

    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);
        for (int pos = 0; pos < (int)count ; ++pos) {
            fprintf(stderr, "count:%d %s theta: %03.2f Dist: %08.2f \n", (int)count,
                (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                nodes[pos].distance_q2/4.0f);
         }
    } else {
        printf("error code: %x\n", ans);
    }

    return ans;
}

int carmen_slamtec_init(carmen_laser_device_t* device __attribute__ ((unused))){
  drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
  if (!drv) {
    fprintf(stderr, "%s: RPLidarDriver create driver failed\n", __func__);
    return 0;
  }

  return 1;
}

int carmen_slamtec_connect(carmen_laser_device_t * device __attribute__ ((unused)), char* filename, int baudrate){
  u_result     op_result;
  rplidar_response_device_health_t healthinfo;
  rplidar_response_device_info_t devinfo;

  do {
    // try to connect
    if (IS_FAIL(drv->connect(filename, baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", filename);
        break;
    }

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            // you can check the detailed failure reason
            fprintf(stderr, "Error, operation time out.\n");
        } else {
            fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
            // other unexpected result
        }
        break;
    }

    // print out the device serial number, firmware and hardware version number..
    fprintf(stderr, "RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        fprintf(stderr, "%02X", devinfo.serialnum[pos]);
    }

    fprintf(stderr, "\n"
            "Version: "RPLIDAR_SDK_VERSION"\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);


    // check the device health
    ////////////////////////////////////////
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        fprintf(stderr, "RPLidar health status : ");
        switch (healthinfo.status) {
        case RPLIDAR_STATUS_OK:
            fprintf(stderr, "OK.");
            break;
        case RPLIDAR_STATUS_WARNING:
            fprintf(stderr, "Warning.");
            break;
        case RPLIDAR_STATUS_ERROR:
            fprintf(stderr, "Error.");
            break;
        }
        fprintf(stderr, " (errorcode: %d)\n", healthinfo.error_code);
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        break;
    }

    if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
        fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
        // enable the following code if you want rplidar to be reboot by software
        // drv->reset();
        break;
    }
  } while(0);

  return 1;
}

int carmen_slamtec_configure(carmen_laser_device_t * device)
{
  if (device->config.laser_type == SLAMTEC_RPLIDAR) {
    //Abe: not sure why FOV and start_angle were read from param file before... but I think this is better
    device->config.fov=RPLIDAR_FOV;
    device->config.start_angle=RPLIDAR_START_ANGLE;
    device->config.angular_resolution = RPLIDAR_ANGULAR_STEP;
    device->config.accuracy = RPLIDAR_ACCURACY;
    device->config.maximum_range = RPLIDAR_MAX_RANGE;
  }

  return 1;
}

int carmen_slamtec_handle_sleep(carmen_laser_device_t* device __attribute__ ((unused)) ){
  sleep(1);
  return 1;
}

int carmen_slamtec_handle(carmen_laser_device_t* device){
  struct timeval timestamp;
  u_result ans;
  rplidar_response_measurement_node_t nodes[360*2];
  size_t   count = _countof(nodes);

#if DEBUG
  printf("waiting for data...\n");
#endif
  // fetech extactly one 0-360 degrees' scan
  ans = drv->grabScanData(nodes, count);
  device->config.angular_resolution = 2*M_PI/count; //RPLIDAR_ANGULAR_STEP;
  if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
    drv->ascendScanData(nodes, count);
#if DEBUG
    for (int pos = 0; pos < (int)count ; ++pos) {
      fprintf(stderr, "count:%d pos:%d %s theta: %03.2f Dist: %08.2f \n", count, pos,
             (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
             (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
             nodes[pos].distance_q2/4.0f);
   }
#endif
   carmen_laser_laser_static_message message;
   message.id = device->laser_id;
   message.config = device->config;
   message.num_readings = count; //reading.n_ranges;
   message.num_remissions = 0;
   gettimeofday(&timestamp, NULL);
   message.timestamp = timestamp.tv_sec + 1e-6*timestamp.tv_usec;
   for (int j=0; j < (int)count; j++){
     message.range[j] = 0.001 * (nodes[(int)count-j-1].distance_q2/4.0f); //reading.ranges[j];
   }
   if (device->f_onreceive!=NULL)
     (*device->f_onreceive)(device, &message);
      return 1;
   }

  return 0;
}

int carmen_slamtec_start(carmen_laser_device_t* device __attribute__ ((unused))){
  drv->startMotor();
  if (IS_FAIL(drv->startScan( /* true */ ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
  {
    fprintf(stderr, "Error, cannot start the scan operation.\n");
  }

  return 1;
}

int carmen_slamtec_stop(carmen_laser_device_t* device){
  UNUSED(device);
  drv->stop();
  drv->stopMotor();

  RPlidarDriver::DisposeDriver(drv);
  return 1;
}



//FIXME I do not want  to malloc the ranges!

int carmen_slamtec_close(struct carmen_laser_device_t* device){
    UNUSED(device);
    return 1; //slamtec_close(hokuyoLaser);
}

carmen_laser_device_t* carmen_create_slamtec_instance(carmen_laser_laser_config_t* config, int laser_id){
  fprintf(stderr,"init slamtec\n");
  carmen_laser_device_t* device=(carmen_laser_device_t*)malloc(sizeof(carmen_laser_device_t));
  carmen_test_alloc(device);
  device->laser_id=laser_id;
  device->config=*config;
  device->f_init=carmen_slamtec_init;
  device->f_connect=carmen_slamtec_connect;
  device->f_configure=carmen_slamtec_configure;
  device->f_start=carmen_slamtec_start;
  device->f_stop=carmen_slamtec_stop;
  device->f_handle=carmen_slamtec_handle;
  device->f_close=carmen_slamtec_close;
  device->f_onreceive=NULL;
  return device;
}

carmen_laser_laser_config_t carmen_slamtec_valid_configs[]=
  {{SLAMTEC_RPLIDAR,	-M_PI,	2*M_PI,	M_PI/180,	6.0,	0.01,	REMISSION_NONE}};


int carmen_slamtec_valid_configs_size=1;

int carmen_init_slamtec_configs(void){
  int i;
  carmen_laser_laser_config_t* conf=carmen_laser_configurations+carmen_laser_configurations_num;
  for (i=0; i<carmen_slamtec_valid_configs_size; i++){
    *conf=carmen_slamtec_valid_configs[i];
    conf++;
  }
  carmen_laser_configurations_num+=carmen_slamtec_valid_configs_size;
  return carmen_laser_configurations_num;
}



