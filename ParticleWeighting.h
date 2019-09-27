#ifndef PARTICLE_WEIGHTING_H
#define PARTICLE_WEIGHTING_H

#include <vector>
#include <cstdint>
#include <math.h>

class ParticleWeighting
{
    std::vector<uint16_t> particles_x;
    std::vector<uint16_t> particles_y;
    std::vector<uint16_t> particles_w;
    uint32_t bound;
    uint32_t sum_weights;
    int16_t max_distance;
    uint16_t max_weight;
    uint16_t min_width;
    uint16_t min_height;
    uint16_t max_width;
    uint16_t max_height;
    uint16_t hue;
    uint16_t sat;
    uint16_t val;
    double factor_h;
    double factor_s;
    double factor_v;
    uint16_t cols;
    uint8_t channels = 3;

public:
    ParticleWeighting(const uint16_t &num_particles_, const double &min_width_, const double &max_width_, const double &min_height_, const double &max_height_, const uint16_t &max_weight_, const uint16_t &cols_, const uint32_t bound_, const std::vector<double> &factors, const std::vector<uint16_t> &hsvBest);
    uint32_t predict_weighting(const uint8_t *pixelPtr);
    bool is_color(const uint8_t *pixelPtr);
    void update();
    void get_particle(const uint16_t &idx_, uint16_t &back_x_, uint16_t &back_y_, uint16_t &back_w_) const;
    void set_particle(const uint16_t &idx_, const uint16_t &change_x_, const uint16_t &change_y_, const uint16_t &change_w_);
    uint32_t getIdx_end_rep() const;
    void setIdx_end_rep(const uint32_t &value);
    int16_t getMax_distance() const;
    void setMax_distance(const int16_t &value);
    uint16_t getMax_weight() const;
    void setMax_weight(const uint16_t &value);
    uint16_t getMin_width() const;
    void setMin_width(const uint16_t &value);
    uint16_t getMin_height() const;
    void setMin_height(const uint16_t &value);
    uint16_t getMax_width() const;
    void setMax_width(const uint16_t &value);
    uint16_t getMax_height() const;
    void setMax_height(const uint16_t &value);
    uint16_t getHue() const;
    void setHue(const uint16_t &value);
    uint16_t getSat() const;
    void setSat(const uint16_t &value);
    uint16_t getVal() const;
    void setVal(const uint16_t &value);
    double getFactor_h() const;
    void setFactor_h(double value);
    double getFactor_s() const;
    void setFactor_s(double value);
    double getFactor_v() const;
    void setFactor_v(double value);
    std::vector<uint16_t> getHsv() const;
    bool setHsv(const std::vector<uint16_t> &hsv);
    std::vector<double> getFactors() const;
    bool setFactors(std::vector<double> &factors);
};

#endif // PARTICLE_WEIGHTING_H
