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

#include "variable.h"

double Variable::GetMaximumKeyValue() const
{
    assert(IsKeyframed());
    return curve_.GetMaximumKeyValue();
}

double Variable::GetMinimumKeyValue() const
{
    assert(IsKeyframed());
    return curve_.GetMinimumKeyValue();
}
