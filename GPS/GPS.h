#ifndef _GPS_H_
#define _GPS_H_

typedef struct {
    s16 degree_latitude;
    s16 degree_longitude;
    double minute_latitude;
    double minute_longitude;
    double altitude;
} GPS_COORDINATE_RAW; // 16 bytes
typedef struct {
    double speed;
    double course;
} GPS_DIRECTION_RAW; // 8 bytes

typedef struct {
    double decimal_degree_latitude;
    double decimal_degree_longitude;
    double altitude;
} GPS_COORDINATE_DEGREES; // 12 bytes

typedef struct {
    us8 hour;
    us8 minute;
    us8 second;
    us16 millisecond;
} GPS_TIME;

typedef struct {
        us8 longitudeString[15];
        us8 latitudeString[15];
        us8 altString[10];
        us8 courseString[10];
        us8 speedString[10];
        us8 longitudeHI;
        us8 latitudeHI;
//        us32 longitudenum;
//        us32 latitudenum;
//        us32 speednum;
//        us32 coursenum;
//        us32 altnum;
        double distnum;
        double waypointdist;
        double waypointcourse;
//        us32 longitudefqueue;
//        us32 latitudefqueue;
//        us32 altfqueue;
//        us32 speedfqueue;
//        us32 coursefqueue;
        us8 ratiotime;
        GPS_TIME time;
        GPS_COORDINATE_RAW coordinate;
        GPS_DIRECTION_RAW direction;
        GPS_COORDINATE_DEGREES coordinate_deg;
} GPS_INFO;

#endif
