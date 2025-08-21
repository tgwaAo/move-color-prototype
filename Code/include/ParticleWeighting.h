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

#ifndef CODE_INCLUDE_PARTICLEWEIGHTING_H_
#define CODE_INCLUDE_PARTICLEWEIGHTING_H_


#include <math.h>
#include <cstdint>
#include <vector>


/**
 * @class ParticleWeighting
 * @date 09/10/19
 * @file ParticleWeighting.h
 * @brief Decide, whether a color is in this area or not via particles.
 */
class ParticleWeighting {
 public:
    /**
     * @brief Constructs a continuous matrix of particles in given area
     * and initializes necessary attributes.
     * @param numParticles Number of particles.
     * @param minWidth Where width starts.
     * @param maxWidth Where width ends.
     * @param minHeight Where height starts.
     * @param maxHeight Where height ends.
     * @param maxWeight Weight of a particle without error.
     * @param cols Number of horizontal pixels.
     * @param bound Minimal number of positive particles.
     * @param factors Factors multiplied by squared error.
     * @param optimalHsvValues HSV values closest to ideal values.
     */
    ParticleWeighting(
        const uint16_t numParticles,
        const double minWidth,
        const double maxWidth,
        const double minHeight,
        const double maxHeight,
        const uint16_t maxWeight,
        const uint16_t cols,
        const uint32_t bound,
        const std::vector<double> &factors,
        const std::vector<double> &optimalHsvValues);

    /**
     * @brief Decide, wheter area contains color or not.
     * @param pixelPtr Pointer pointing to current image.
     * @return Decision of containing color.
     */
    bool isColor(const uint8_t *const pixelPtr);

    /**
     * @brief Update position of particles after changed size.
     */
    void update();

    /**
     * @brief Get values of a particle.
     * @param idx Index of wanted particle.
     * @param backX X position of particle.
     * @param backY Y position of particle.
     * @param backW Weight of particle.
     * @param alright Is alright, if idx_ is in valid range.
     */
    void getParticle(
        const uint16_t idx,
        uint16_t backX,
        uint16_t backY,
        uint16_t backW,
        bool alright) const;

    /**
     * @brief Set values of a particle.
     * @param idx Index of interesting particle.
     * @param changeX Next x position of particle.
     * @param changeY Next y position of particle.
     * @param changeW Next weight of particle.
     */
    void setParticle(
        const uint16_t idx,
        const uint16_t changeX,
        const uint16_t changeY,
        const uint16_t changeW);

    /**
     * @brief Get sum of weights.
     * @return Sum of weights.
     */
    uint32_t getSumWeights() const;

    /**
     * @brief Set sum of weights.
     * @param sumWeights New sum of weights.
     */
    void setSumWeights(const uint32_t sumWeights);

    /**
     * @brief Get weight of no error.
     * @return Weight of no error.
     */
    uint16_t getMaxWeight() const;

    /**
     * @brief Set maximum possible weight.
     * @param maxWeight Maximum possible weight.
     */
    void setMaxWeight(const uint16_t maxWeight);

    /**
     * @brief Get horizontal start of area.
     * @return Horizontal start of area.
     */
    uint16_t getMinWidth() const;

    /**
     * @brief Set horizontal start of area.
     * @param minWidth Horizontal start of area.
     */
    void setMinWidth(const uint16_t minWidth);

    /**
     * @brief Get vertical start of area.
     * @return Vertical start of area.
     */
    uint16_t getMinHeight() const;

    /**
     * @brief Set vertical start of area.
     * @param minHeight Vertical start of area.
     */
    void setMinHeight(const uint16_t minHeight);

    /**
     * @brief Get horizontal end of area.
     * @return Horizontal end of area.
     */
    uint16_t getMaxWidth() const;

    /**
     * @brief Set horizontal end of area.
     * @param maxWidth Horizontal end of area.
     */
    void setMaxWidth(const uint16_t maxWidth);

    /**
     * @brief Get vertical end of area.
     * @return Vertical end of area.
     */
    uint16_t getMaxHeight() const;

    /**
     * @brief Set vertical end of area.
     * @param maxHeight Vertical end of area.
     */
    void setMaxHeight(const uint16_t maxHeight);

    /**
     * @brief Get vector of optimal hsv values.
     * @return Vector of optimal hsv values.
     */
    std::vector<double> getHsv() const;

    /**
     * @brief Set optimal hsv values.
     * @param hsv Optimal hsv values.
     * @return Returns true, if input vector has a length of 3.
     */
    bool setHsv(const std::vector<double> &hsv);

    /**
     * @brief Get factors of squared errors of hsv values.
     * @return Factors of squared errors of hsv values.
     */
    std::vector<double> getFactors() const;

    /**
     * @brief Set factors of squared errors of hsv values.
     * @param factors Factors of squared errors of hsv values.
     * @return Returns true, if input vector has a length of 3.
     */
    bool setFactors(const std::vector<double> &factors);

 private:
    const uint8_t HSV_CHANNELS = 3;

    std::vector<uint16_t> particlesX;
    std::vector<uint16_t> particlesY;
    std::vector<uint16_t> particlesW;
    uint32_t boundary;
    uint32_t sumWeights;
    uint16_t maxWeight;
    uint16_t minWidth;
    uint16_t minHeight;
    uint16_t maxWidth;
    uint16_t maxHeight;
    uint16_t cols;
    uint16_t hue;
    uint16_t sat;
    uint16_t val;
    double factorH;
    double factorS;
    double factorV;
};

#endif  // CODE_INCLUDE_PARTICLEWEIGHTING_H_
