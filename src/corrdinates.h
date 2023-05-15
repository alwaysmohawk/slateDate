#ifndef COORDINATES_H
#define COORDINATES_H

#include <Arduino.h>

#define EarthRadiusKm 6371.0
#define MetersPerKm 1000.0

class Coordinate
{
private:
    float lat;
    float lon;
    bool set;
public:
    Coordinate();
    Coordinate(float,float);
    ~Coordinate();

    float Lat();
    float Lon();

    float LatRadian();
    float LonRadian();
};

Coordinate::Coordinate(float Lat, float Lon)
{
    this->lat = lat;
    this->lon = Lon;
    this->set = true;
}

Coordinate::Coordinate()
{
    this->lat = 0;
    this->lon = 0;
    this->set = false;
}

Coordinate::~Coordinate()
{
}

float Coordinate::Lat(){
    return this->lat;
}

float Coordinate::Lon(){
    return this->lon;
}

float Coordinate::LatRadian(){
    return this->lat;
}

float Coordinate::LonRadian(){
    return this->lon;
}

float DistanceBetweenCoordinates(Coordinate Alpha, Coordinate Bravo) {
    
    float dlat = Bravo.LatRadian() - Alpha.LatRadian();  // Difference in latitude
    float dlon = Bravo.LonRadian() - Alpha.LonRadian();     

    // Where did this come from, cannot read. Should link to site
    // TODO: "a" is a poor name
    // TODO: "c" is a poor name
    float a = pow(sin(dlat / 2), 2) + cos(Alpha.LatRadian()) * cos(Bravo.LatRadian()) * pow(sin(dlon / 2), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = EarthRadiusKm * c * MetersPerKm;  // Calculating distance in meters

  return distance;
}

struct DistanceResult{
    int index;
    float distance;
};

DistanceResult ClosestWaypoint(Coordinate coor, Objective Waypoints[], int indexSize){
    float closes = 10000000000;
    float tmpDistance;
    
    DistanceResult result;
    for (int x=0; x<indexSize, x++;){
        Objective currentWaypoint = Waypoints[x];
        tmpDistance = DistanceBetweenCoordinates(coor,currentWaypoint.GetCoordinate());
        if ( tmpDistance < closes) {
            result.index = x;
            result.distance = tmpDistance;
        }
    }
    return result;
};

#endif