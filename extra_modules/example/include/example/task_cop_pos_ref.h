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
/// @brief Task describing the CoP position following a reference value in the form of { Ax = b }
class HUMOTO_LOCAL TaskCoPPosRef : public humoto::TaskAB
{
  private:
    /// @brief Reference trajectory of the CoP
    Eigen::VectorXd zRef_;

  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAB)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskAB::setDefaults();
        setGain(1);
    }

    /// @brief Finalizes the class initialization
    void finalize() { TaskAB::finalize(); }

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskCoPPosRef(const double gain = 1) : TaskAB("TaskCoPPosRef", gain) {}

    /// @brief Forms the matrices A and b to represent the task
    ///
    /// @param sol_structure structure of the problems solution
    /// @param model_base model (can be downcasted dynamically to a specific model type if necessary)
    /// @param control_problem control_problem (can be downcasted dynamically to a specific problem type if necessary)
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        // Downcast the control problem into a MPCVerticalMotion type
        const humoto::example::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::example::MPCVerticalMotion &>(control_problem);

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &b = getB();

        // Setup the reference trajectory along the preview horizon (it is the middle of the bounds)
        if (zRef_.size() != 6 * (long)mpc.getPreviewHorizonLength()) zRef_.resize(6 * mpc.getPreviewHorizonLength());

        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            double xMin = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            double xMax = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            double yMin = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            double yMax = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
            double z = mpc.stepPlan().z()(mpc.currentStepIndex() + i) + mpc.pbParams().h_CoM_;
            zRef_(6 * i) = 0.5 * (xMin + xMax);
            zRef_(6 * i + 1) = 0.5 * (yMin + yMax);
            zRef_(6 * i + 2) = z;
            zRef_(6 * i + 3) = 0.5 * (xMin + xMax);
            zRef_(6 * i + 4) = 0.5 * (yMin + yMax);
            zRef_(6 * i + 5) = z;
        }

        // Compute the A and b matrices
        A.noalias() = getGain() * mpc.Ou();
        b.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zRef_);
    };
};
}
}
