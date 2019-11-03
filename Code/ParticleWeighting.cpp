#include "ParticleWeighting.h"

void ParticleWeighting::setSumWeights(const uint32_t& value)
{
    sumWeights = value;
}

uint32_t ParticleWeighting::getSumWeights() const
{
    return sumWeights;
}

uint16_t ParticleWeighting::getMaxWeight() const
{
    return maxWeight_;
}

void ParticleWeighting::setMaxWeight(const uint16_t &value)
{
    maxWeight_ = value;
}

uint16_t ParticleWeighting::getMinWidth() const
{
    return minWidth_;
}

void ParticleWeighting::setMinWidth(const uint16_t &value)
{
    minWidth_ = value;
}

uint16_t ParticleWeighting::getMinHeight() const
{
    return minHeight_;
}

void ParticleWeighting::setMinHeight(const uint16_t &value)
{
    minHeight_ = value;
}

uint16_t ParticleWeighting::getMaxWidth() const
{
    return maxWidth_;
}

void ParticleWeighting::setMaxWidth(const uint16_t &value)
{
    maxWidth_ = value;
}

uint16_t ParticleWeighting::getMaxHeight() const
{
    return maxHeight_;
}

void ParticleWeighting::setMaxHeight(const uint16_t &value)
{
    maxHeight_ = value;
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

double ParticleWeighting::getFactorH() const
{
    return factorH;
}

void ParticleWeighting::setFactorH(double value)
{
    factorH = value;
}

double ParticleWeighting::getFactorS() const
{
    return factorS;
}

void ParticleWeighting::setFactorS(double value)
{
    factorS = value;
}

double ParticleWeighting::getFactorV() const
{
    return factorV;
}

void ParticleWeighting::setFactorV(double value)
{
    factorV = value;
}

ParticleWeighting::ParticleWeighting(const uint16_t &numParticles, const double &minWidth, const double &maxWidth, const double &minHeight, const double &maxHeight, const uint16_t &maxWeight, const uint16_t &cols, const uint32_t bound, const std::vector<double> &factors, const std::vector<uint16_t> &hsvBest)
{
    minWidth_ = minWidth;
    maxWidth_ = maxWidth;
    minHeight_ = minHeight;
    maxHeight_ = maxHeight;
    maxWeight_ = maxWeight;
    factorH = factors[0];
    factorS = factors[1];
    factorV = factors[2];
    hue = hsvBest[0];
    sat = hsvBest[1];
    val = hsvBest[2];
    cols_ = cols;
    bound_ = bound;
    hsvChannels = 3;

    particlesX.resize(numParticles);
    particlesY.resize(numParticles);
    particlesW.resize(numParticles);

    update();
}

uint32_t ParticleWeighting::calculateSumWeights(const uint8_t * const pixelPtr)
{
    int16_t v_h;
    int16_t v_s;
    int16_t v_v;
    double prediction;
    sumWeights = 0;

    for (int i = 0; i < particlesX.size(); ++i) {
        //        calculate probability of being correct
        v_h =  hue - pixelPtr[particlesY[i]*cols_*hsvChannels + particlesX[i]*hsvChannels + 0];
        v_s =  sat - pixelPtr[particlesY[i]*cols_*hsvChannels + particlesX[i]*hsvChannels + 1];
        v_v =  val - pixelPtr[particlesY[i]*cols_*hsvChannels + particlesX[i]*hsvChannels + 2];

        prediction = 1 / (1 + pow(v_h,2)*factorH + pow(v_s,2)*factorS+ pow(v_v,2)*factorV);

        if (prediction < 0.5)
            particlesW[i] = 1;
        else
            particlesW[i] = maxWeight_;

        sumWeights += particlesW[i];
    }

    return sumWeights;
}

void ParticleWeighting::update()
{
    uint16_t diff = maxWidth_ - minWidth_;
    uint32_t num_pixels = diff * (maxHeight_-minHeight_);
    uint16_t step = num_pixels / particlesX.size();

    for (int i = 0; i < particlesX.size(); ++i) {
        particlesX[i] = ((i*step) % int(diff)) + minWidth_;
        particlesY[i] = ((i*step) / diff) + minHeight_;
        particlesW[i] = 1;
    }
}

bool ParticleWeighting::is_color(const uint8_t * const pixelPtr)
{
    if (calculateSumWeights(pixelPtr) < bound_)
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
    factors[0] = factorH;
    factors[1] = factorS;
    factors[2] = factorV;
    return factors;
}

bool ParticleWeighting::setFactors(std::vector<double> &factors)
{
    if (factors.size() == hsvChannels) {
        factorH = factors[0];
        factorS = factors[1];
        factorV = factors[2];
        return true;
    } else {
        return false;
    }
}
