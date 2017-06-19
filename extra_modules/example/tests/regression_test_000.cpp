/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iostream>
#include <limits>
#include <iomanip>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
#include "humoto/humoto.h"
#include "humoto/example.h"
#include "humoto/qpoases.h"

//testing
#include "gtest/gtest.h"


#include "utilities.h"


namespace humoto
{
    namespace example
    {
        class TestFixture : public ::testing::Test
        {
            protected:
                humoto::OptimizationProblem             opt_problem;

                humoto::qpoases::Solver                 solver;
                humoto::qpoases::Solution               solution;
                humoto::qpoases::SolverParameters       solver_parameters;

                humoto::example::WalkParameters             walk_parameters;
                humoto::walking::StanceFiniteStateMachine   stance_fsm;

                humoto::example::Model                      model;
                humoto::example::ModelState                 model_state;
                humoto::example::MPCforWPG                  wpg;


            protected:
                virtual void SetUp() {}


                TestFixture()
                {
                    solver_parameters.crash_on_any_failure_ = false;

                    solver.setParameters(solver_parameters);
                    stance_fsm.setParameters(walk_parameters);

                    setupHierarchy_v0(opt_problem);
                }


                /**
                 * @brief Simple control loop
                 *
                 * @return Last solution vector
                 */
                bool run()
                {
                    for (unsigned int i = 0;; ++i)
                    {
                        // prepare control problem for new iteration
                        if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
                        {
                            break;
                        }

                        // form an optimization problem
                        opt_problem.form(solution, model, wpg);

                        // solve an optimization problem
                        solver.solve(solution, opt_problem);

                        // extract next model state from the solution and update model
                        stance_fsm.shiftTime(100);
                        model_state = wpg.getNextModelState(solution, stance_fsm, model);
                        model.updateState(model_state);

                        break;
                    }

                    return (true);
                }
        };


        TEST_F(TestFixture, SolutionMatchReferenceNoSolutionNoASGuess)
        {
            ASSERT_TRUE(run());
        }
    }
}



/**
 * @brief main
 *
 * @param[in] argc number of args
 * @param[in] argv args
 *
 * @return status
 */
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
