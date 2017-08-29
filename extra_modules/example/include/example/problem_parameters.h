/**
    @file
    @author  Stanislas Brossette
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
   public:
    double g_;                                                // gravity
    double h_CoM_;                                            // height of center of mass
    double t_;                                                // length of one time step
    size_t n_;                                                // number of time steps in horizon
    size_t nIterations_;                                      // total Number of iterations to reach endTime_
    double endTime_;                                          // End time of the control
    etools::Vector3 comVelRef_;                               // target com speed
    std::vector<std::vector<double> > leftStepsParameters_;   // parameters of the right stepping plan
    std::vector<std::vector<double> > rightStepsParameters_;  // parameters of the right stepping plan

    double gainTaskVelocity_;   // gain of the CoM velocity task
    double gainTaskMinJerk_;    // gain of the Min Jerk task
    double gainTaskCoPBounds_;  // gain of the CoP bounds task
    double gainTaskCoPPosRef_;  // gain of the CoP position reference task

   protected:
#define HUMOTO_CONFIG_SECTION_ID "ProblemParameters"
#define HUMOTO_CONFIG_ENTRIES                      \
    HUMOTO_CONFIG_SCALAR_(g);                      \
    HUMOTO_CONFIG_SCALAR_(h_CoM);                  \
    HUMOTO_CONFIG_SCALAR_(t);                      \
    HUMOTO_CONFIG_SCALAR_(n);                      \
    HUMOTO_CONFIG_SCALAR_(endTime);                \
    HUMOTO_CONFIG_COMPOUND_(comVelRef);            \
    HUMOTO_CONFIG_COMPOUND_(leftStepsParameters);  \
    HUMOTO_CONFIG_COMPOUND_(rightStepsParameters); \
    HUMOTO_CONFIG_SCALAR_(gainTaskVelocity);       \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPPosRef);      \
    HUMOTO_CONFIG_SCALAR_(gainTaskMinJerk);        \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPBounds);
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

   public:
    HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(ProblemParameters)

    /**
     * @brief Default constructor
     */
    ProblemParameters() { setDefaults(); }

    void finalize() { nIterations_ = endTime_ / t_; }

    /**
     * @brief Default parameters of the walk
     */
    void setDefaults()
    {
        g_ = 9.81;
        h_CoM_ = 0.8;
        t_ = 0.005;
        n_ = 100;
        endTime_ = 10;
        comVelRef_(0) = 0.1;
        comVelRef_(1) = 0;
        comVelRef_(2) = 0;
        gainTaskVelocity_ = 100;
        gainTaskCoPBounds_ = 100;
        gainTaskCoPPosRef_ = 100;
        gainTaskMinJerk_ = 100;
        setDefaultStepParameters();
    }

    void setDefaultStepParameters()
    {
        double l0[] = {0, 0.10, 0.0, 0.0, 2.00};
        double r0[] = {0, -0.10, 0.0, 0.0, 2.99};
        double l1[] = {0.2, 0.10, 0.0, 2.91, 3.99};
        double r1[] = {0.4, -0.10, 0.0, 3.91, 4.99};
        double l2[] = {0.6, 0.10, 0.0, 4.91, 5.99};
        double r2[] = {0.8, -0.10, 0.0, 5.91, 7.99};
        double l3[] = {1.0, 0.10, 0.0, 7.91, 8.99};
        double r3[] = {1.3, -0.10, 0.0, 8.91, 13.0};
        double l4[] = {1.6, 0.10, 0.0, 9.91, 13.0};
        leftStepsParameters_.push_back(std::vector<double>(l0, l0 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r0, r0 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l1, l1 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r1, r1 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l2, l2 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r2, r2 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l3, l3 + 5));
        rightStepsParameters_.push_back(std::vector<double>(r3, r3 + 5));
        leftStepsParameters_.push_back(std::vector<double>(l4, l4 + 5));
    }
};
}
}

