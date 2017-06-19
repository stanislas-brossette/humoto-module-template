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
#include "humoto/qpoases.h"
#include "humoto/example.h"

#include "utilities.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);


/**
 * @brief Simple control loop
 *
 * @param[in] argc number of arguments
 * @param[in] argv arguments
 *
 * @return status
 */
int main(int argc, char **argv)
{
    try
    {
        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem               opt_problem;

        humoto::qpoases::SolverParameters         solver_parameters;
        solver_parameters.crash_on_any_failure_ = false;
        humoto::qpoases::Solver                   solver(solver_parameters);
        humoto::qpoases::Solution                 solution;


        humoto::example::WalkParameters             walk_parameters;
        humoto::walking::StanceFiniteStateMachine   stance_fsm(walk_parameters);


        // model representing the controlled system
        humoto::example::Model                      model;
        humoto::example::ModelState                 model_state;


        // control problem, which is used to construct an optimization problem
        humoto::example::MPCforWPG                  wpg;


        setupHierarchy_v0(opt_problem);


        for (unsigned int i = 0;; ++i)
        {
            // -------------------------------------------------
            if (wpg.update(model, stance_fsm, walk_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }
            opt_problem.form(solution, model, wpg);
            solver.solve(solution, opt_problem);
            // -------------------------------------------------

            // -------------------------------------------------
            stance_fsm.shiftTime(100);
            model_state = wpg.getNextModelState(solution, stance_fsm, model);
            model.updateState(model_state);
            // -------------------------------------------------

            break;
        }
        HUMOTO_LOG_RAW("Done.");
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }
    return (0);
}

