#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <math.h>
#include "corrdinates.h"


class Objective
{
private:
    bool visited;
    int ledPosition;
public:
    // make coordinate public so it is easier to compare distances
    Coordinate cord;
    
    Objective(float, float, int);
    ~Objective();
    
    Coordinate GetCoordinate();
    void SetAsVisited();
    bool Visited();

};

Objective::Objective(float Lat, float Lon, int LedPosition){
    this->visited = false;
    this->cord = Coordinate(Lat,Lon);
    this->ledPosition = LedPosition;
}


Coordinate Objective::GetCoordinate(){
    return this->cord;
}

void Objective::SetAsVisited(){
    this->visited = true;
}

bool Objective::Visited(){
    return this->visited;
}

Objective::~Objective()
{
}

#endif