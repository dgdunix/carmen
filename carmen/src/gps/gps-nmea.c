#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include <carmen/carmen.h>
#include <carmen/gps_nmea_interface.h>

#include "gps.h"
#include "gps-io.h"

int
gps_parse_gga( char * line, int num_chars )
{
  char * ptr;
  int    i;
  for (i=1; i<num_chars-1; i++) {
    if (line[i]=='$' || line[i]=='*')
      return(FALSE);
  }
  if (num_chars>0 && carmen_extern_gpgga_ptr!=NULL) {

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->utc = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->latitude = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->lat_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->longitude = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->long_orient = ptr[0];

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->gps_quality = atoi(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->num_satellites = atoi(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->hdop = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->sea_level = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->altitude = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->geo_sea_level = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->geo_sep = atof(ptr);

    ptr = strsep( &line, ",");
    if (ptr==NULL) return(FALSE);
    carmen_extern_gpgga_ptr->data_age = atoi(ptr);

    return(TRUE);
  }
  return(FALSE);
}

int
carmen_gps_parse_data( char * line, int num_chars )
{
#ifdef GPGGA_PARSE_DEBUG
  int  i;
#endif
#ifdef PARSE_DEBUG
  int  j;
#endif
  if (num_chars>=6) {
    if (!strncmp( "$GPGGA", line, 6 ) ) {
#ifdef PARSE_DEBUG
      fprintf( stderr, "(GPGGA)" );
#endif
#ifdef GPGGA_PARSE_DEBUG
      fprintf( stderr, "(" );
      for (i=0;i<num_chars;i++)
	fprintf( stderr, "%c", line[i] );
      fprintf( stderr, ")" );
#endif
      return(gps_parse_gga( line, num_chars ));
    } else if (!strncmp( "$GPGLL", line, 6 ) ) {
#ifdef PARSE_DEBUG
      fprintf( stderr, "(GPGLL)" );
#endif
    } else if (!strncmp( "$GPRMC", line, 6 ) ) {
#ifdef PARSE_DEBUG
      fprintf( stderr, "(GPRMC)" );
#endif
    } else if (!strncmp( "$GPGSV", line, 6 ) ) {
#ifdef PARSE_DEBUG
      fprintf( stderr, "(GPGSV)" );
#endif
    } else if (!strncmp( "$GPGSA", line, 6 ) ) {
#ifdef PARSE_DEBUG
      fprintf( stderr, "(GPGSA)" );
#endif
    } else {
#ifdef PARSE_DEBUG
      fprintf( stderr, "[" );
      j=1;
      while (j<num_chars  && line[j]!=',' && line[j]!='*') {
	fprintf( stderr, "%c", line[j] );
	j++;
      }
      fprintf( stderr, "]" );
#endif
    }
  }
  return(FALSE);
}

