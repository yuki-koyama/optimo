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

#ifndef UTIL_H
#define UTIL_H

#include <string>

namespace Util
{
    /// Generate a sampling from the uniform distribution of [0, 1]
    double GenerateUniformReal();
    
    /// Generate a sampling from the normal distribution
    double GenerateStandardNormal();
    
    std::string GetCurrentTimeInString();
}

#endif // UTIL_H
