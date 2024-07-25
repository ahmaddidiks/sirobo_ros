#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double last_pos = 0;
    double vel = 0;
    double last_vel;
    double rads_per_count = 0;
    double ppr;
    double radius = 0.08;
    double acc;
    double distance;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
      ppr = counts_per_rev;
    }

    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

    void calculate_wheel_position(double dt)
    {
      // calculate position from encoder
      // pos += calc_enc_angle();
      pos += (enc / ppr) * (M_PI*radius); // cm

      // calculate vel from pos
      vel = (pos - last_pos) / dt;

      // calculate acc from vel
      acc = (vel - last_vel) / dt;

      last_pos = pos;
      last_vel = vel;
      distance = pos;
    }
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
