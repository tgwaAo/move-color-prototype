#include "ParticleWeighting.h"

void ParticleWeighting::setSumWeights(const uint32_t& sum_weights)
{
    this->sum_weights = sum_weights;
}

uint32_t ParticleWeighting::getSumWeights() const
{
    return sum_weights;
}

uint16_t ParticleWeighting::getMax_weight() const
{
    return max_weight;
}

void ParticleWeighting::setMax_weight(const uint16_t &value)
{
    max_weight = value;
}

uint16_t ParticleWeighting::getMin_width() const
{
    return min_width;
}

void ParticleWeighting::setMin_width(const uint16_t &value)
{
    min_width = value;
}

uint16_t ParticleWeighting::getMin_height() const
{
    return min_height;
}

void ParticleWeighting::setMin_height(const uint16_t &value)
{
    min_height = value;
}

uint16_t ParticleWeighting::getMax_width() const
{
    return max_width;
}

void ParticleWeighting::setMax_width(const uint16_t &value)
{
    max_width = value;
}

uint16_t ParticleWeighting::getMax_height() const
{
    return max_height;
}

void ParticleWeighting::setMax_height(const uint16_t &value)
{
    max_height = value;
}

uint16_t ParticleWeighting::getHue() const
{
    return hue;
}

void ParticleWeighting::setHue(const uint16_t &value)
{
    hue = value;
}

uint16_t ParticleWeighting::getSat() const
{
    return sat;
}

void ParticleWeighting::setSat(const uint16_t &value)
{
    sat = value;
}

uint16_t ParticleWeighting::getVal() const
{
    return val;
}

void ParticleWeighting::setVal(const uint16_t &value)
{
    val = value;
}

double ParticleWeighting::getFactor_h() const
{
    return factor_h;
}

void ParticleWeighting::setFactor_h(double value)
{
    factor_h = value;
}

double ParticleWeighting::getFactor_s() const
{
    return factor_s;
}

void ParticleWeighting::setFactor_s(double value)
{
    factor_s = value;
}

double ParticleWeighting::getFactor_v() const
{
    return factor_v;
}

void ParticleWeighting::setFactor_v(double value)
{
    factor_v = value;
}

ParticleWeighting::ParticleWeighting(const uint16_t &num_particles_, const double &min_width_, const double &max_width_, const double &min_height_, const double &max_height_, const uint16_t &max_weight_, const uint16_t &cols_, const uint32_t bound_, const std::vector<double> &factors, const std::vector<uint16_t> &hsvBest)
{
    min_width = min_width_;
    max_width = max_width_;
    min_height = min_height_;
    max_height = max_height_;
    max_weight = max_weight_;
    factor_h = factors[0];
    factor_s = factors[1];
    factor_v = factors[2];
    hue = hsvBest[0];
    sat = hsvBest[1];
    val = hsvBest[2];
    cols = cols_;
    bound = bound_;
    hsvChannels = 3;

    particles_x.resize(num_particles_);
    particles_y.resize(num_particles_);
    particles_w.resize(num_particles_);

    update();
}

uint32_t ParticleWeighting::calculateSumWeights(const uint8_t *pixelPtr)
{
    int16_t v_h;
    int16_t v_s;
    int16_t v_v;
    double prediction;
    sum_weights = 0;

    for (int i = 0; i < particles_x.size(); ++i) {
        //        calculate probability of being correct
        v_h =  hue - pixelPtr[particles_y[i]*cols*hsvChannels + particles_x[i]*hsvChannels + 0];
        v_s =  sat - pixelPtr[particles_y[i]*cols*hsvChannels + particles_x[i]*hsvChannels + 1];
        v_v =  val - pixelPtr[particles_y[i]*cols*hsvChannels + particles_x[i]*hsvChannels + 2];

        prediction = 1 / (1 + pow(v_h,2)*factor_h + pow(v_s,2)*factor_s+ pow(v_v,2)*factor_v);

        if (prediction < 0.5)
            particles_w[i] = 1;
        else
            particles_w[i] = max_weight;

        sum_weights += particles_w[i];
    }

    return sum_weights;
}

void ParticleWeighting::update()
{
    uint16_t diff = max_width - min_width;
    uint32_t num_pixels = diff * (max_height-min_height);
    uint16_t step = num_pixels / particles_x.size();

    for (int i = 0; i < particles_x.size(); ++i) {
        particles_x[i] = ((i*step) % int(diff)) + min_width;
        particles_y[i] = ((i*step) / diff) + min_height;
        particles_w[i] = 1;
    }
}

bool ParticleWeighting::is_color(const uint8_t *pixelPtr)
{
    if (calculateSumWeights(pixelPtr) < bound)
        return false;
    else
        return true;
}

std::vector<uint16_t> ParticleWeighting::getHsv() const
{
    std::vector<uint16_t> hsv(3,0);
    hsv[0] = hue;
    hsv[1] = sat;
    hsv[2] = val;
    return hsv;
}

bool ParticleWeighting::setHsv(const std::vector<uint16_t> &hsv)
{
    if (hsv.size() == hsvChannels) {
        hue = hsv[0];
        sat = hsv[1];
        val = hsv[2];
        return true;
    } else {
        return false;
    }
}

std::vector<double> ParticleWeighting::getFactors() const
{
    std::vector<double> factors(3,0);
    factors[0] = factor_h;
    factors[1] = factor_s;
    factors[2] = factor_v;
    return factors;
}

bool ParticleWeighting::setFactors(std::vector<double> &factors)
{
    if (factors.size() == hsvChannels) {
        factor_h = factors[0];
        factor_s = factors[1];
        factor_v = factors[2];
        return true;
    } else {
        return false;
    }
}
