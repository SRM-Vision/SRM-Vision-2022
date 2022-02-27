/**
 * Radar station definition header.
 * \author trantuan-20048607
 * \date 2022.1.28
 * \attention It's recommended to include battlefield.h for complete function.
 */

#ifndef RADAR_STATION_H_
#define RADAR_STATION_H_

#include "../facility.h"

class RadarStation : public Facility {
public:
    explicit RadarStation(Colors color) :
            Facility(color, 0, kRadarStation) {}
};

#endif  // RADAR_STATION_H_
