/**
    @file
    @author  Stanislas Brossette
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace humoto
{
    namespace example
    {
        /**
         * @brief [task_cvel.m]
         */
        class HUMOTO_LOCAL TaskCoMVelocity: public humoto::TaskAB
        {
            protected:
                #define HUMOTO_CONFIG_ENTRIES \
                    HUMOTO_CONFIG_PARENT_CLASS(TaskAB);
                #include HUMOTO_CONFIG_DEFINE_ACCESSORS


                void setDefaults()
                {
                    TaskAB::setDefaults();
                    setGain(10);
                }


                void finalize()
                {
                    TaskAB::finalize();
                }


            public:
                TaskCoMVelocity(const double gain = 100) : TaskAB("TaskCoMVelocity", gain)
                {
                }


                /// @copydoc humoto::TaskBase::form
                void form(  const humoto::SolutionStructure &sol_structure,
                            const humoto::Model &model_base,
                            const humoto::ControlProblem &control_problem)
                {
                    const humoto::example::SimpleMPC  &mpc = dynamic_cast <const humoto::example::SimpleMPC &> (control_problem);


                    Eigen::MatrixXd &A = getA();
                    Eigen::VectorXd &b = getB();

                    Eigen::VectorXd cvel_ref;
                    cvel_ref.resize(mpc.getPreviewHorizonLength()*3);

                    for (std::size_t i = 0; i < mpc.getPreviewHorizonLength(); ++i)
                    {
                        cvel_ref.segment(i*3, 3) = mpc.pbParams().cVelRef_;
                    }

                    A.noalias() = getGain() * (mpc.velocity_selector() * mpc.Uu());

                    b.noalias() = -getGain() * (mpc.velocity_selector() * mpc.Ux() * mpc.currentState()  - cvel_ref);
                };
        };
    }
}
