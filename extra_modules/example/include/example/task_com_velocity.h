/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
namespace example
{
/// @brief Task describing the CoM velocity following a reference value in the form of { Ax = b }
class HUMOTO_LOCAL TaskCoMVelocity : public humoto::TaskAB
{
  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskAB::setDefaults();
        setGain(100);
    }

    /// @brief Finalizes the class initialization
    void finalize() { TaskAB::finalize(); }

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskCoMVelocity(const double gain = 100) : TaskAB("TaskCoMVelocity", gain) {}

    /// @brief Forms the matrices A and b to represent the task
    ///
    /// @param sol_structure structure of the problems solution
    /// @param model_base model (can be downcasted dynamically to a specific model type if necessary)
    /// @param control_problem control_problem (can be downcasted dynamically to a specific problem type if necessary)
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        // Downcast the control problem into a simpleMPC type
        const humoto::example::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::example::MPCVerticalMotion &>(control_problem);

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &b = getB();

        // Setup the reference velocity along the preview horizon
        Eigen::VectorXd cvel_ref;
        if (cvel_ref.size() != (long)mpc.getPreviewHorizonLength() * 3)
            cvel_ref.resize(mpc.getPreviewHorizonLength() * 3);
        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            cvel_ref.segment(i * 3, 3) = mpc.pbParams().comVelRef_;
        }

        // Compute the A and b matrices
        A.noalias() = getGain() * (mpc.velocity_selector() * mpc.Uu());
        b.noalias() = -getGain() * (mpc.velocity_selector() * mpc.Ux() * mpc.currentState() - cvel_ref);
    };
};
}
}
