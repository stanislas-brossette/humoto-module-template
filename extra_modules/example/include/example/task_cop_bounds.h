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
class HUMOTO_LOCAL TaskCoPBounds : public humoto::TaskALU
{
   private:
    Eigen::VectorXd zBoundsLow_;
    Eigen::VectorXd zBoundsHigh_;

   protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskALU);
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    void setDefaults()
    {
        TaskALU::setDefaults();
        setGain(1);
    }

    void finalize() { TaskALU::finalize(); }

   public:
    TaskCoPBounds(const double gain = 1) : TaskALU("TaskCoPBounds", gain) {}

    /// @copydoc humoto::TaskBase::form
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        const humoto::example::SimpleMPC &mpc = dynamic_cast<const humoto::example::SimpleMPC &>(control_problem);

        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &l = getLowerBounds();
        Eigen::VectorXd &u = getUpperBounds();

        if (zBoundsHigh_.size() != 2 * (long)mpc.getPreviewHorizonLength())
            zBoundsHigh_.resize(2 * mpc.getPreviewHorizonLength());
        if (zBoundsLow_.size() != 2 * (long)mpc.getPreviewHorizonLength())
            zBoundsLow_.resize(2 * mpc.getPreviewHorizonLength());

        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            zBoundsLow_(2 * i) = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            zBoundsLow_(2 * i + 1) = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            zBoundsHigh_(2 * i) = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            zBoundsHigh_(2 * i + 1) = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
        }

        A.noalias() = getGain() * mpc.Ou();

        l.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zBoundsLow_);
        u.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zBoundsHigh_);
    };
};
}
}
