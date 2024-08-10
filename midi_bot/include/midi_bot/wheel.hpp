#ifndef WHEEL_HPP_
#define WHEEL_HPP_

#include <string>
#include <cmath>

class Wheel
{
    public: 
    std::string name = "";
    int enc{0};
    double cmd{0};
    double pos{0};
    double vel{0};
    double rads_per_count{0};
    
    Wheel() = default;
    Wheel(const std::string &wheel_name, int counts_per_rev);

    void setup(const std::string &Wheel_name, int counts_per_rev);
    double calc_enc_angle();
};

#endif //WHEEL_HPP_
