/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#pragma  once

namespace humoto
{
    /**
     * @brief Example: example module
     *
     * @ingroup Modules
     * @{
     * @defgroup example example
     * @}
     *
     * @ingroup example
     */
    namespace example
    {
    }
}

#include "humoto/walking.h"

#include "tools/history.h"
#include "example/common.h"
#include "example/model-state.h"
#include "example/model.h"
#include "example/simple-mpc.h"
#include "example/task_comvelocity.h"
#include "example/task_copbounds.h"
#include "example/task_minjerk.h"
#include "example/setup_hierarchy.h"
