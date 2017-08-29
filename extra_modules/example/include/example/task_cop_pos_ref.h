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
class HUMOTO_LOCAL TaskCoPPosRef : public humoto::TaskAB
{
   private:
    Eigen::VectorXd zRef_;

   protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAB);
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    void setDefaults()
    {
        TaskAB::setDefaults();
        setGain(1);
    }

    void finalize() { TaskAB::finalize(); }

   public:
    TaskCoPPosRef(const double gain = 1) : TaskAB("TaskCoPPosRef", gain) {}

    /// @copydoc humoto::TaskBase::form
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        const humoto::example::SimpleMPC &mpc = dynamic_cast<const humoto::example::SimpleMPC &>(control_problem);

        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &b = getB();
        if (zRef_.size() != 2 * (long)mpc.getPreviewHorizonLength()) zRef_.resize(2 * mpc.getPreviewHorizonLength());

        for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
        {
            double xMin = mpc.stepPlan().xMin()(mpc.currentStepIndex() + i);
            double xMax = mpc.stepPlan().xMax()(mpc.currentStepIndex() + i);
            double yMin = mpc.stepPlan().yMin()(mpc.currentStepIndex() + i);
            double yMax = mpc.stepPlan().yMax()(mpc.currentStepIndex() + i);
            zRef_(2 * i) = 0.5 * (xMin + xMax);
            zRef_(2 * i + 1) = 0.5 * (yMin + yMax);
        }

        A.noalias() = getGain() * mpc.Ou();
        b.noalias() = getGain() * (-mpc.Ox() * mpc.currentState() + zRef_);
    };
};
}
}
