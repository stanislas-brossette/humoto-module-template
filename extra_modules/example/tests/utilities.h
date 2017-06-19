/**
    @file
    @author  Alexander Sherikov

    @copyright 2017 INRIA. Licensed under the Apache License, Version 2.0. (see
    LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


/**
 * @brief Hierarchy 0
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v0(humoto::OptimizationProblem &opt_problem)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer   task_infeasible     (new humoto::TaskInfeasibleInequality);
    humoto::TaskSharedPointer   task_zero           (new humoto::TaskZeroVariables);

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_infeasible, 0);
    opt_problem.pushTask(task_zero, 1);
}
