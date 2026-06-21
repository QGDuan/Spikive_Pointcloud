#ifndef TRANSFER_HPP
#define TRANSFER_HPP

#include "LocalGeographicCS.hpp"

class coord_transfer {
   public:
    LocalGeographicCS cs;
    coord_transfer();
    coord_transfer(double lat, double lon);
    ~coord_transfer();

    void set_origin(double lat, double lon);

   private:
    double origin_lat;  ///< 投影原点纬度.
    double origin_lon;  ///< 投影原点经度.
};

coord_transfer::coord_transfer() : origin_lat(0), origin_lon(0) {}

coord_transfer::coord_transfer(double lat, double lon) {
    set_origin(lat, lon);
}

void coord_transfer::set_origin(double lat, double lon) {
    origin_lat = lat;
    origin_lon = lon;
    cs.set_origin(origin_lat, origin_lon);
}
coord_transfer::~coord_transfer() {}

#endif