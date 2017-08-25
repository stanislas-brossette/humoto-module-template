
#pragma once

namespace humoto
{
    namespace example
    {
        class HUMOTO_LOCAL SimpleMPC : public humoto::MPC
        {
            private:
                // Matrices to update the state
                etools::Matrix9 A_;
                etools::Matrix9x3 B_;
                Eigen::MatrixXd Ux_;
                Eigen::MatrixXd Uu_;
                etools::Vector9 currentState_;
                const ProblemParameters& pbParams_;
                etools::SelectionMatrix velocity_selector_;
                StateHistory stateHistory_;

                etools::Matrix9 computeA()
                {
                  double t = pbParams_.t_;
                  Eigen::Matrix3d Ablock;
                  Ablock.setIdentity();
                  Ablock(0,1) = t;
                  Ablock(0,2) = t*t/2.0;
                  Ablock(1,2) = t;
                  A_.setIdentity();
                  A_.block(0,0,3,3) = Ablock;
                  A_.block(3,3,3,3) = Ablock;
                  A_.block(6,6,3,3) = Ablock;
                  return A_;
                }
                etools::Matrix9x3 computeB()
                {
                  double t = pbParams_.t_;
                  Eigen::Vector3d Bblock;
                  Bblock(0) = t*t*t/6.0;
                  Bblock(1) = t*t/2.0;
                  Bblock(2) = t;

                  B_.setZero();
                  B_.block(0,0,3,1) = Bblock;
                  B_.block(3,1,3,1) = Bblock;
                  B_.block(6,2,3,1) = Bblock;
                  return B_;
                }
            public:
                /**
                 * @brief Constructor
                 */
                SimpleMPC(const ProblemParameters& pbParam)
                  : pbParams_(pbParam),
                  velocity_selector_(3,1),
                  stateHistory_(pbParams_.t_)
                {
                  computeA();
                  computeB();
                  condenseTimeInvariant(Ux_, Uu_, pbParams_.n_, A_, B_);
                }

                const ProblemParameters& pbParams() const {return pbParams_;}
                const etools::SelectionMatrix& velocity_selector() const {return velocity_selector_;}
                const Eigen::MatrixXd& Uu() const {return Uu_;}
                const Eigen::MatrixXd& Ux() const {return Ux_;}
                const etools::Vector9& currentState() const { return currentState_;}
                const StateHistory& stateHistory()const{return stateHistory_;}
                



                /**
                 * @brief Update control problem
                 *
                 * @param[in] model     model of the system
                 * @param[in] problem_parameters
                 *
                 * @return ControlProblemStatus::OK/ControlProblemStatus::STOPPED
                 */
                ControlProblemStatus::Status
                    update( const humoto::example::Model                      &model,
                            const humoto::example::ProblemParameters          &problem_parameters)
                {
                    sol_structure_.reset();
                    sol_structure_.addSolutionPart("JERK_VARIABLE_ID", problem_parameters.n_*3 );
                    std::size_t number_of_state_variables = model.Ns_ * problem_parameters.n_;

                    currentState_(0) = model.state_.com_state_.position_(0);
                    currentState_(1) = model.state_.com_state_.velocity_(0);
                    currentState_(2) = model.state_.com_state_.acceleration_(0);
                    currentState_(3) = model.state_.com_state_.position_(1);
                    currentState_(4) = model.state_.com_state_.velocity_(1);
                    currentState_(5) = model.state_.com_state_.acceleration_(1);
                    currentState_(6) = model.state_.com_state_.position_(2);
                    currentState_(7) = model.state_.com_state_.velocity_(2);
                    currentState_(8) = model.state_.com_state_.acceleration_(2);


                    return(ControlProblemStatus::OK);
                }


                /**
                 * @brief Get next model state.
                 * Computes Xˆ_k+1 = A.Xˆ_k + B.dddX_k
                 *
                 * @param[in] solution  solution
                 * @param[in] model model
                 *
                 * @return next model state.
                 */
                humoto::example::ModelState   getNextModelState(
                        const humoto::Solution                          &solution,
                        const humoto::example::Model                      &model)
                {
                    humoto::example::ModelState state;
                    currentState_(0) = model.state_.com_state_.position_(0);
                    currentState_(1) = model.state_.com_state_.velocity_(0);
                    currentState_(2) = model.state_.com_state_.acceleration_(0);
                    currentState_(3) = model.state_.com_state_.position_(1);
                    currentState_(4) = model.state_.com_state_.velocity_(1);
                    currentState_(5) = model.state_.com_state_.acceleration_(1);
                    currentState_(6) = model.state_.com_state_.position_(2);
                    currentState_(7) = model.state_.com_state_.velocity_(2);
                    currentState_(8) = model.state_.com_state_.acceleration_(2);

                    etools::Vector9 newState;
                    newState = A_*currentState_ + B_*solution.x_.segment(0,3);
                    state.com_state_.position_(0) = newState(0);
                    state.com_state_.velocity_(0) = newState(1);
                    state.com_state_.acceleration_(0) = newState(2);
                    state.com_state_.position_(1) = newState(3);
                    state.com_state_.velocity_(1) = newState(4);
                    state.com_state_.acceleration_(1) = newState(5);
                    state.com_state_.position_(2) = newState(6);
                    state.com_state_.velocity_(2) = newState(7);
                    state.com_state_.acceleration_(2) = newState(8);

                    stateHistory_.addStateAndControl(newState, solution.x_.segment(0,3));

                    return(state);
                }

                size_t getPreviewHorizonLength() const
                {
                  return pbParams_.n_;
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

