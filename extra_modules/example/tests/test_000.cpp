/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <iomanip>
#include <iostream>
#include <limits>

#define HUMOTO_GLOBAL_LOGGER_ENABLED

// Enable YAML configuration files (must be first)
#include "humoto/config_yaml.h"
#include "humoto/humoto.h"
#include "humoto/qpoases.h"
#include "humoto/example.h"

HUMOTO_INITIALIZE_GLOBAL_LOGGER(std::cout);

/// @brief Implementation and resolution of the problem described in test_000.yaml

int main()
{
    // All the execution is in a try scope in order to be able to catch exceptions
    try
    {
        std::string config_file_name = "extra_modules/example/tests/test_000.yaml";

        // yaml configuration file reader
        humoto::config::yaml::Reader config_reader(config_file_name);

        // optimization problem (a stack of tasks / hierarchy)
        humoto::OptimizationProblem opt_problem;

        // parameters of the solver
        humoto::qpoases::SolverParameters solver_parameters;
        solver_parameters.crash_on_any_failure_ = false;

        // Actual solver (initialized with solver_parameters)
        humoto::qpoases::Solver solver(solver_parameters);

        // Structure that will contain the solutions
        humoto::qpoases::Solution solution;

        // Problems parameters
        humoto::example::ProblemParameters problem_parameters(config_reader);

        // model and state representing the controlled system
        humoto::example::Model model;
        humoto::example::ModelState model_state;

        // control problem, which is used to construct an optimization problem
        humoto::example::SimpleMPC simpleMPC(problem_parameters);

        // Populate the optimization problem
        setupHierarchy_v0(opt_problem, problem_parameters);

        for (unsigned int i = 0; i < problem_parameters.nIterations_; ++i)
        {
            // if update fails, exit
            if (simpleMPC.update(model, problem_parameters) != humoto::ControlProblemStatus::OK)
            {
                break;
            }
            // form the optimization problem with the current state of the model
            opt_problem.form(solution, model, simpleMPC);
            // Solve the problem and put the result in solution
            solver.solve(solution, opt_problem);

            // Update the model and its state
            model_state = simpleMPC.getNextModelState(solution, model);
            model.updateState(model_state);
        }
        // Write the python file plotFile.py
        simpleMPC.logger().plot();

        // Execute the command { python plotFile.py } in terminal
        std::string command = "python plotFile.py";
        system(command.c_str());

        // All Done
        HUMOTO_LOG_RAW("Done.");
    }
    catch (const std::exception &e)
    {
        // Log the caught exceptions
        HUMOTO_LOG_RAW(e.what());
        exit(-1);
    }
    return (0);
}

