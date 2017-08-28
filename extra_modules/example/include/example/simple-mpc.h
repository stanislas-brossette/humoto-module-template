
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
                etools::Matrix2x9 D_;
                etools::Matrix2x3 E_;
                Eigen::MatrixXd Ux_;
                Eigen::MatrixXd Uu_;
                Eigen::MatrixXd Ox_;
                Eigen::MatrixXd Ou_;
                etools::Vector9 currentState_;
                const ProblemParameters& pbParams_;
                etools::SelectionMatrix velocity_selector_;
                StateHistory stateHistory_;
                StepPlan stepPlan_;
                size_t currentStepIndex_;

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
                etools::Matrix2x9 computeD()
                {
                  double t = pbParams_.t_;
                  D_.setZero();
                  D_(0,0) = 1;
                  D_(0,1) = t;
                  D_(0,2) = t*t/2.0 - pbParams_.h_CoM_/pbParams_.g_;
                  D_(1,3) = 1;
                  D_(1,4) = t;
                  D_(1,5) = t*t/2.0 - pbParams_.h_CoM_/pbParams_.g_;
                  return D_;
                }
                etools::Matrix2x3 computeE()
                {
                  double t = pbParams_.t_;
                  E_.setZero();
                  E_(0,0) = t*t*t/6 - pbParams_.h_CoM_*t/pbParams_.g_;
                  E_(1,1) = t*t*t/6 - pbParams_.h_CoM_*t/pbParams_.g_;
                  return E_;
                }
            public:
                /**
                 * @brief Constructor
                 */
                SimpleMPC(const ProblemParameters& pbParam)
                  : pbParams_(pbParam),
                  velocity_selector_(3,1),
                  stateHistory_(pbParams_.t_),
                  stepPlan_(pbParams_.leftSteps_, pbParams_.rightSteps_, pbParams_.t_),
                  currentStepIndex_(0)
                {
                  stateHistory_.setMinMax(stepPlan_.xMin(), stepPlan_.xMax(),
                                          stepPlan_.yMin(), stepPlan_.yMax(),
                                          stepPlan_.zMin(), stepPlan_.zMax());
                  computeA();
                  computeB();
                  computeD();
                  computeE();
                  condenseTimeInvariant(Ux_, Uu_, pbParams_.n_, A_, B_);
                  condenseOutput(Ox_, Ou_, D_, E_, Ux_, Uu_);
                }

                const ProblemParameters& pbParams() const {return pbParams_;}
                const etools::SelectionMatrix& velocity_selector() const {return velocity_selector_;}
                const Eigen::MatrixXd& Uu() const {return Uu_;}
                const Eigen::MatrixXd& Ux() const {return Ux_;}
                const Eigen::MatrixXd& Ou() const {return Ou_;}
                const Eigen::MatrixXd& Ox() const {return Ox_;}
                const etools::Vector9& currentState() const { return currentState_;}
                const StateHistory& stateHistory()const{return stateHistory_;}
                const StepPlan& stepPlan() const {return stepPlan_;}
                

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
                    //std::size_t number_of_state_variables = model.Ns_ * problem_parameters.n_;

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

                    std::cout << "currentStepIndex_: " << currentStepIndex_ << std::endl;
                    currentStepIndex_++;

                    return(state);
                }

                size_t getPreviewHorizonLength() const
                {
                  return pbParams_.n_;
                }
                size_t currentStepIndex() const {return currentStepIndex_;}


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

