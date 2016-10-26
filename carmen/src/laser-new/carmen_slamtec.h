#ifndef CARMEN_SLAMTEC_H
#define CARMEN_SLAMTEC_H
#include "carmen_laser_device.h"

#ifdef __cplusplus
extern "C" {
#endif
carmen_laser_device_t* carmen_create_slamtec_instance(carmen_laser_laser_config_t* config, int laser_id);

int carmen_init_slamtec_configs(void);
#ifdef __cplusplus
}
#endif
#endif

