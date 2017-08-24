/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace example
    {
        /**
         * @brief Class containing options
         */
        class HUMOTO_LOCAL ProblemParameters : public humoto::config::ConfigurableBase
        {
            protected:
                #define HUMOTO_CONFIG_SECTION_ID "ProblemParameters"
                #define HUMOTO_CONFIG_ENTRIES
                #include "humoto/config/define_accessors.h"


            public:
                HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(ProblemParameters)


                /**
                 * @brief Default constructor
                 */
                ProblemParameters()
                {
                    setDefaults();
                }


                /**
                 * @brief Default parameters of the walk
                 */
                void setDefaults()
                {
                  g_ = 9.81;
                  h_CoM_ = 0.5;
                  t_ = 0.01;
                  n_ = 20;
                  cVelRef_(0) = -1;
                  cVelRef_(1) = 1;
                  cVelRef_(2) = 0.5;
                }

            public:
                double g_; // gravity
                double h_CoM_; // height of center of mass
                double t_; // length of one time step
                size_t n_; // number of time steps in horizon
                etools::Vector3 cVelRef_; // target com horizontal speed
        };
    }
}

