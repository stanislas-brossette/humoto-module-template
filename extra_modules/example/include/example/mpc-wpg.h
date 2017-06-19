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
        class HUMOTO_LOCAL MPCforWPG : public humoto::MPC
        {
            public:
                /**
                 * @brief Constructor
                 */
                MPCforWPG()
                {
                }


                /**
                 * @brief Update control problem
                 *
                 * @param[in] model     model of the system
                 * @param[in] stance_fsm  walking finite state machine
                 * @param[in] walk_parameters
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                ControlProblemStatus::Status
                    update( const humoto::example::Model                      &model,
                            const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                            const humoto::example::WalkParameters             &walk_parameters)
                {
                    sol_structure_.reset();
                    sol_structure_.addSolutionPart("EXAMPLE", 10);

                    return(ControlProblemStatus::OK);
                }


                /**
                 * @brief Get next model state.
                 *
                 * @param[in] solution  solution
                 * @param[in] stance_fsm  walking finite state machine
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::example::ModelState   getNextModelState(
                        const humoto::Solution                          &solution,
                        const humoto::walking::StanceFiniteStateMachine &stance_fsm,
                        const humoto::example::Model                      &model)
                {
                    humoto::example::ModelState state;
                    return(state);
                }


                /**
                 * @brief Log
                 *
                 * @param[in,out] logger logger
                 * @param[in] parent parent
                 * @param[in] name name
                 */
                void log(   humoto::Logger & logger HUMOTO_GLOBAL_LOGGER_IF_DEFINED,
                            const LogEntryName &parent = LogEntryName(),
                            const std::string &name = "mpcwpg") const
                {
                    LogEntryName subname = parent;
                    subname.add(name);
                }
        };
    }
}

