// Modified MIT License
//
// Copyright (c) 2019 tgwaAo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modified part:
//
// THIS SOFTWARE DOES NOT CHECK YOUR SURROUNDINGS NOR DOES IT CONTROL YOUR
// MOVEMENT, SO IT IS UNDER YOUR OWN RESPONSIBILITY TO ENSURE NOBODY GETS HURT
// AND NOTHING GETS DAMAGED. PLAY CAREFULLY AND CHECK YOUR SURROUNDINGS BEFORE
// PLAYING.

/**
 * ParticleWeighting.cpp
 *
 * Class to calculate decision of containing a color in hsv space or not.
 * This is done via unmoving particles, because the color
 * from one pixel and the one next to it will not change often.
 */

#include "ParticleWeighting.h"


ParticleWeighting::ParticleWeighting(
    const uint16_t numParticles,
    const double minWidth,
    const double maxWidth,
    const double minHeight,
    const double maxHeight,
    const uint16_t maxWeight,
    const uint16_t cols,
    const uint32_t bound,
    const std::vector<double> &factors,
    const std::vector<double> &optimalHsvValues
) {
    this->minWidth = minWidth;
    this->maxWidth = maxWidth;
    this->minHeight = minHeight;
    this->maxHeight = maxHeight;
    this->maxWeight = maxWeight;

    if (factors.size() == HSV_CHANNELS) {
        factorH = factors[0];
        factorS = factors[1];
        factorV = factors[2];
    }

    if (optimalHsvValues.size() == HSV_CHANNELS) {
        hue = optimalHsvValues[0];
        sat = optimalHsvValues[1];
        val = optimalHsvValues[2];
    }

    this->cols = cols;
    this->boundary = bound;

    particlesX.resize(numParticles);
    particlesY.resize(numParticles);
    particlesW.resize(numParticles);

    update();
}

bool ParticleWeighting::isColor(const uint8_t *const pixelPtr) {
    int16_t errHue;
    int16_t errSat;
    int16_t errVal;

    double prediction;
    sumWeights = 0;
    uint32_t failureIdx = particlesX.size() - boundary;

    for (uint32_t idx = 0; idx < particlesX.size(); ++idx) {
        // calculate probability of being correct
        errHue =
            hue - pixelPtr[
         particlesY[idx] * cols * HSV_CHANNELS
         + particlesX[idx] * HSV_CHANNELS];

        errSat =
            sat - pixelPtr[
         particlesY[idx] * cols * HSV_CHANNELS
         + particlesX[idx] * HSV_CHANNELS + 1];

        errVal =
            val - pixelPtr[
         particlesY[idx] * cols * HSV_CHANNELS
         + particlesX[idx] * HSV_CHANNELS + 2];

        prediction = 1
                     / (1
                        + pow(errHue, 2) * factorH
                        + pow(errSat, 2) * factorS
                        + pow(errVal, 2) * factorV);

        if (prediction < 0.5)
            particlesW[idx] = 0;
        else
            particlesW[idx] = maxWeight;

        sumWeights += particlesW[idx];

        if (boundary <= sumWeights)
            return true;

        uint32_t leftParticles = particlesX.size() - idx;
        uint64_t possiblePositives = leftParticles * maxWeight;
        uint32_t neededPositives = boundary - sumWeights;

        if (possiblePositives < neededPositives)
            return false;
    }
    return false;
}

void ParticleWeighting::update() {
    uint16_t diff = maxWidth - minWidth;
    uint32_t num_pixels = diff * (maxHeight - minHeight);
    uint16_t step = num_pixels / particlesX.size();

    for (uint32_t i = 0; i < particlesX.size(); ++i) {
        particlesX[i] = ((i*step) % static_cast<int>(diff)) + minWidth;
        particlesY[i] = ((i*step) / diff) + minHeight;
        particlesW[i] = 1;
    }
}

std::vector<double> ParticleWeighting::getHsv() const {
    std::vector<double> hsv(3, 0);
    hsv[0] = hue;
    hsv[1] = sat;
    hsv[2] = val;
    return hsv;
}

bool ParticleWeighting::setHsv(const std::vector<double> &hsv) {
    if (hsv.size() == HSV_CHANNELS) {
        hue = hsv[0];
        sat = hsv[1];
        val = hsv[2];
        return true;
    } else {
        return false;
    }
}

std::vector<double> ParticleWeighting::getFactors() const {
    return std::vector<double> {factorH, factorS, factorV};
}

bool ParticleWeighting::setFactors(const std::vector<double> &factors) {
    if (factors.size() == HSV_CHANNELS) {
        factorH = factors[0];
        factorS = factors[1];
        factorV = factors[2];
        return true;
    } else {
        return false;
    }
}

void ParticleWeighting::getParticle(
    const uint16_t idx,
    uint16_t& backX,
    uint16_t& backY,
    uint16_t& backW,
    bool& alright) const {
    if (idx < particlesX.size()) {
        backX = particlesX[idx];
        backY = particlesY[idx];
        backW = particlesW[idx];
        alright = true;
    } else {
        backX = 0;
        backY = 0;
        backW = 0;
        alright = false;
    }
}

void ParticleWeighting::setSumWeights(const uint32_t sumWeights) {
    this->sumWeights = sumWeights;
}

uint32_t ParticleWeighting::getSumWeights() const {
    return sumWeights;
}

uint16_t ParticleWeighting::getMaxWeight() const {
    return maxWeight;
}

void ParticleWeighting::setMaxWeight(const uint16_t maxWeight) {
    this->maxWeight = maxWeight;
}

uint16_t ParticleWeighting::getMinWidth() const {
    return minWidth;
}

void ParticleWeighting::setMinWidth(const uint16_t minWidth) {
    this->minWidth = minWidth;
}

uint16_t ParticleWeighting::getMinHeight() const {
    return minHeight;
}

void ParticleWeighting::setMinHeight(const uint16_t minHeight) {
    this->minHeight = minHeight;
}

uint16_t ParticleWeighting::getMaxWidth() const {
    return maxWidth;
}

void ParticleWeighting::setMaxWidth(const uint16_t maxWidth) {
    this->maxWidth = maxWidth;
}

uint16_t ParticleWeighting::getMaxHeight() const {
    return maxHeight;
}

void ParticleWeighting::setMaxHeight(const uint16_t maxHeight) {
    this->maxHeight = maxHeight;
}
