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

        humoto::example::ProblemParameters             problem_parameters;

        // model representing the controlled system
        humoto::example::Model                      model;
        humoto::example::ModelState                 model_state;


        // control problem, which is used to construct an optimization problem
        humoto::example::SimpleMPC                  control_problem(problem_parameters);


        setupHierarchy_v0(opt_problem);


        for (unsigned int i = 0; i < 1000; ++i)
        {
            // -------------------------------------------------
            if (control_problem.update(model, problem_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }
            opt_problem.form(solution, model, control_problem);
            solver.solve(solution, opt_problem);
            //std::cout << "Solution: " << std::endl;
            //solution.log();
            // -------------------------------------------------

            // -------------------------------------------------
            //stance_fsm.shiftTime(100);
            model_state = control_problem.getNextModelState(solution, model);
            model.updateState(model_state);
            // -------------------------------------------------
        }
        control_problem.stateHistory().plot();
        std::string command = "python plotFile.py";
        system(command.c_str());
        HUMOTO_LOG_RAW("Done.");
    }
    catch (const std::exception &e)
    {
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }
    return (0);
}

