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

#ifndef VARIABLE_H
#define VARIABLE_H

#include "curve.h"

class Variable
{
public:
    Variable(double var = 0.0) : var_(var) {}
    double var_;
    Curve curve_;
    bool IsKeyframed() const { return curve_.GetControlPoints().size() != 0; }
    double GetValue(double t) const { if (IsKeyframed()) return curve_.GetValue(t); else return var_; }
    int GetNumOfControlPoints() const { return curve_.GetControlPoints().size(); }
    
    double GetMaximumKeyValue() const;
    double GetMinimumKeyValue() const;
};

#endif // VARIABLE_H
