/*
 *  OptiMo
 *
 *  Copyright (c) 2018 National Institute of Advanced Industrial Science and Technology (AIST)
 *  Authors of this file: Yuki Koyama <koyama.y@aist.go.jp>
 *
 *  This program is dual-licensed; You may use it under either LGPLv3 or
 *  our commercial license. See the LICENSE files for details.
 *
 */

#include "util.h"
#include <cmath>
#include <random>
#include <iomanip>
#include <sstream>

namespace
{
    std::random_device seed;
    std::default_random_engine gen(seed());
    std::uniform_real_distribution<double> uniform_dist(0.0, 1.0);
    std::normal_distribution<> normal_dist(0.0, 1.0);
}

namespace Util
{
    double GenerateUniformReal()
    {
        return uniform_dist(gen);
    }
    
    double GenerateStandardNormal()
    {
        return normal_dist(gen);
    }

    std::string GetCurrentTimeInString()
    {
        const std::time_t t = std::time(nullptr);
        std::stringstream s; s << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");
        return s.str();
    }
}
