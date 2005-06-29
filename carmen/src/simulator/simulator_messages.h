/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef SIMULATOR_MESSAGES_H
#define SIMULATOR_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  carmen_point_t pose;
  double timestamp;
  char host[10];
} carmen_simulator_set_truepose_message;

#define CARMEN_SIMULATOR_SET_TRUEPOSE_NAME "carmen_simulator_set_truepose"
#define CARMEN_SIMULATOR_SET_TRUEPOSE_FMT  "{{double,double,double},double,[char:10]}"

typedef struct {
  carmen_point_t truepose;
  carmen_point_t odometrypose;
  double timestamp;
  char host[10];
} carmen_simulator_truepos_message;

#define      CARMEN_SIMULATOR_TRUEPOS_NAME       "carmen_simulator_truepos"
#define      CARMEN_SIMULATOR_TRUEPOS_FMT        "{{double,double,double},{double,double,double},double,[char:10]}"

typedef struct {
  char *other_central;
  double timestamp;
  char host[10];
} carmen_simulator_connect_robots_message;

#define CARMEN_SIMULATOR_CONNECT_ROBOTS_NAME "carmen_simulator_connect_robots"
#define CARMEN_SIMULATOR_CONNECT_ROBOTS_FMT  "{string,double,[char:10]}"

typedef struct {
  carmen_point_t pose;
  double speed;
  carmen_simulator_object_t type;
  double timestamp;
  char host[10];
} carmen_simulator_set_object_message;

#define CARMEN_SIMULATOR_SET_OBJECT_NAME "carmen_simulator_set_object"
#define CARMEN_SIMULATOR_SET_OBJECT_FMT  "{{double,double,double},double,int,double,[char:10]}"

typedef struct {
  int num_objects;
  carmen_traj_point_t *objects_list;
  double timestamp;
  char host[10];
} carmen_simulator_objects_message;

#define CARMEN_SIMULATOR_OBJECTS_NAME "carmen_simulator_objects"
#define CARMEN_SIMULATOR_OBJECTS_FMT  "{int,<{double,double,double,double,double}:1>,double,[char:10]}"

typedef struct {
  double timestamp;
  char host[10];
} carmen_simulator_clear_objects_message;

#define CARMEN_SIMULATOR_CLEAR_OBJECTS_NAME "carmen_simulator_clear_objects"
#define CARMEN_SIMULATOR_CLEAR_OBJECTS_FMT "{double,[char:10]}"

typedef struct {
  double timestamp;
  char host[10];
} carmen_simulator_query_message;

#define CARMEN_SIMULATOR_NEXT_TICK_NAME       "carmen_simulator_next_tick"
#define CARMEN_SIMULATOR_NEXT_TICK_FMT        "{double,[char:10]}"
  
#define CARMEN_SIMULATOR_TRUEPOS_QUERY_NAME   "carmen_simulator_truepos_query"
#define CARMEN_SIMULATOR_TRUEPOS_QUERY_FMT    "{double,[char:10]}"
  
#define CARMEN_SIMULATOR_OBJECTS_QUERY_NAME   "carmen_simulator_objects_query"
#define CARMEN_SIMULATOR_OBJECTS_QUERY_FMT    "{double,[char:10]}"

#define CARMEN_SIMULATOR_CLEAR_OBJECTS_NAME   "carmen_simulator_clear_objects"
#define CARMEN_SIMULATOR_CLEAR_OBJECTS_FMT    "{double,[char:10]}"


#ifdef __cplusplus
}
#endif

#endif