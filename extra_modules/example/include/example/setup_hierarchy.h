/**
    @file
    @author  Stanislas Brossette
    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


/**
 * @brief Hierarchy 0: basic
 *
 * @param[in] opt_problem       hierarchy
 */
void setupHierarchy_v0(humoto::OptimizationProblem &opt_problem, const humoto::example::ProblemParameters& params)
{
    // tasks, which are used in the control problem
    humoto::TaskSharedPointer task_cop_bounds(new humoto::example::TaskCoPBounds(params.gainTaskCoPBounds_));
    humoto::TaskSharedPointer task_cop_pos_ref(new humoto::example::TaskCoPPosRef(params.gainTaskCoPPosRef_));
    humoto::TaskSharedPointer task_com_velocity(new humoto::example::TaskCoMVelocity(params.gainTaskVelocity_));
    humoto::TaskSharedPointer task_min_jerk(new humoto::TaskZeroVariables(params.gainTaskMinJerk_));

    // reset the optimization problem
    opt_problem.reset(2);

    // push tasks into the stack/hierarchy
    opt_problem.pushTask(task_cop_bounds, 0);
    opt_problem.pushTask(task_cop_pos_ref, 1);
    opt_problem.pushTask(task_com_velocity, 1);
    opt_problem.pushTask(task_min_jerk, 1);
}


