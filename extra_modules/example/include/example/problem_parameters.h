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
/// @brief Class containing the parameters of the problem. It can be configured through yaml file
class HUMOTO_LOCAL ProblemParameters : public humoto::config::RelaxedConfigurableBase
{
   public:
    /// @brief Gravity constant
    double g_;
    /// @brief height of center of mass
    double h_CoM_;

    /// @brief Min value of the quantity { zeta = (c_z - p_z) / (ddc_z + g_z) }
    double zetaMin_;
    /// @brief Max value of the quantity { zeta = (c_z - p_z) / (ddc_z + g_z) }
    double zetaMax_;

    /// @brief length of one time step
    double t_;
    /// @brief number of time steps in horizon
    size_t n_;
    /// @brief total number of iterations to reach endTime_
    size_t nIterations_;
    /// @brief End time of the control
    double endTime_;

    /// @brief target com speed
    etools::Vector3 comVelRef_;
    /// @brief target com height
    double comHeightRef_;

    /// @brief parameters of the right stepping plan
    std::vector<std::vector<double> > leftStepsParameters_;
    /// @brief parameters of the right stepping plan
    std::vector<std::vector<double> > rightStepsParameters_;

    /// @brief gain of the CoM velocity task
    double gainTaskVelocity_;
    /// @brief gain of the CoM Height task
    double gainTaskCoMHeight_;
    /// @brief gain of the Min Jerk task
    double gainTaskMinJerk_;
    /// @brief gain of the CoP bounds task
    double gainTaskCoPBounds_;
    /// @brief gain of the CoP position reference task
    double gainTaskCoPPosRef_;

   protected:
/// Those macros define the necessary tools to read the variables from a yaml configuration file
#define HUMOTO_CONFIG_SECTION_ID "ProblemParameters"
#define HUMOTO_CONFIG_ENTRIES                      \
    HUMOTO_CONFIG_SCALAR_(g);                      \
    HUMOTO_CONFIG_SCALAR_(h_CoM);                  \
    HUMOTO_CONFIG_SCALAR_(zetaMin);                  \
    HUMOTO_CONFIG_SCALAR_(zetaMax);                  \
    HUMOTO_CONFIG_SCALAR_(t);                      \
    HUMOTO_CONFIG_SCALAR_(n);                      \
    HUMOTO_CONFIG_SCALAR_(endTime);                \
    HUMOTO_CONFIG_COMPOUND_(comVelRef);            \
    HUMOTO_CONFIG_SCALAR_(comHeightRef);            \
    HUMOTO_CONFIG_COMPOUND_(leftStepsParameters);  \
    HUMOTO_CONFIG_COMPOUND_(rightStepsParameters); \
    HUMOTO_CONFIG_SCALAR_(gainTaskVelocity);       \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPPosRef);      \
    HUMOTO_CONFIG_SCALAR_(gainTaskMinJerk);        \
    HUMOTO_CONFIG_SCALAR_(gainTaskCoPBounds);
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

   public:
    HUMOTO_DEFINE_CONFIG_CONSTRUCTORS(ProblemParameters)

    /// @brief Default constructor
    ProblemParameters() { setDefaults(); }

    /// @brief Finalizes the configuration of this class after all other parameters have been set
    void finalize() { nIterations_ = endTime_ / t_; }

    /// @brief Sets the default parameters of the walk
    void setDefaults()
    {
        g_ = 9.81;
        h_CoM_ = 0.8;
        zetaMin_ = -10;
        zetaMax_ = 10;
        t_ = 0.005;
        n_ = 100;
        endTime_ = 10;
        comVelRef_(0) = 0.1;
        comVelRef_(1) = 0;
        comVelRef_(2) = 0;
        gainTaskVelocity_ = 100;
        gainTaskCoMHeight_ = 100;
        gainTaskCoPBounds_ = 100;
        gainTaskCoPPosRef_ = 100;
        gainTaskMinJerk_ = 100;
        setDefaultStepParameters();
    }

    /// @brief Sets the default values of the stepping plan
    void setDefaultStepParameters()
    {
        //Create the default arrays of step parameters (x,y,z,tMin,tMax)
        double l0[] = {0, 0.10, 0.0, 0.0, 2.00};
        double r0[] = {0, -0.10, 0.0, 0.0, 2.99};
        double l1[] = {0.2, 0.10, 0.0, 2.91, 3.99};
        double r1[] = {0.4, -0.10, 0.0, 3.91, 4.99};
        double l2[] = {0.6, 0.10, 0.0, 4.91, 5.99};
        double r2[] = {0.8, -0.10, 0.0, 5.91, 7.99};
        double l3[] = {1.0, 0.10, 0.0, 7.91, 8.99};
        double r3[] = {1.3, -0.10, 0.0, 8.91, 30.0};
        double l4[] = {1.6, 0.10, 0.0, 9.91, 30.0};
        //Populate step plan parameters vectors with parameters
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

