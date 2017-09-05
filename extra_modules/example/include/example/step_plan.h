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
class HUMOTO_LOCAL Step
{
  public:
    /// @brief Constructor for step
    ///
    /// @param x x
    /// @param y y
    /// @param z z
    /// @param tMin starting time of the step
    /// @param tMax end time of the step
    Step(double x, double y, double z, double tMin, double tMax) : x_(x), y_(y), z_(z), tMin_(tMin), tMax_(tMax) {}

    /// @brief Construstor for the step from a vector
    ///
    /// @param v vector containing [ x, y, z, tMin, tMax]
    Step(const std::vector<double> v) : x_(v.at(0)), y_(v.at(1)), z_(v.at(2)), tMin_(v.at(3)), tMax_(v.at(4)) {}

    /// @brief Getter for x
    const double& x() const { return x_; }
    /// @brief Getter for y
    const double& y() const { return y_; }
    /// @brief Getter for z
    const double& z() const { return z_; }
    /// @brief Getter for tMin
    const double& tMin() const { return tMin_; }
    /// @brief Getter for tMax
    const double& tMax() const { return tMax_; }

  private:
    double x_;
    double y_;
    double z_;
    double tMin_;
    double tMax_;
};

class HUMOTO_LOCAL StepPlan
{
  public:
    /// @brief Default constructor
    StepPlan(){};

    /// @brief Constructor from list of parameters
    ///
    /// @param leftStepsParameters parameters for all the left foot steps
    /// @param rightStepsParameters parameters for all the right foot steps
    /// @param T timestep
    StepPlan(const std::vector<std::vector<double> >& leftStepsParameters,
             const std::vector<std::vector<double> >& rightStepsParameters, double T)
        : T_(T)
    {
        for (size_t i = 0; i < leftStepsParameters.size(); ++i)
        {
            HUMOTO_ASSERT(leftStepsParameters.at(i).size() == 5,
                          "[Config] each step parameter must be a size 5 vector]")
            leftSteps_.push_back(Step(leftStepsParameters.at(i)));
        }
        for (size_t i = 0; i < rightStepsParameters.size(); ++i)
        {
            HUMOTO_ASSERT(rightStepsParameters.at(i).size() == 5,
                          "[Config] each step parameter must be a size 5 vector]")
            rightSteps_.push_back(Step(rightStepsParameters.at(i)));
        }
        computePlan();
    }

    /// @brief Computes the values of xMin, xMax, yMin, yMax, z (Sustentation polygon) for each time step
    ///
    /// @param leftSteps list of left foot steps
    /// @param rightSteps list of right foot steps
    void computePlan(std::vector<Step> leftSteps, std::vector<Step> rightSteps)
    {
        leftSteps_ = leftSteps;
        rightSteps_ = rightSteps;
        tMax_ = leftSteps[0].tMax();
        stepXWidth_ = 0.2;
        stepYWidth_ = 0.1;
        stepZWidth_ = 0.0;

        for (size_t i = 0; i < leftSteps_.size(); ++i)
            if (leftSteps_[i].tMax() > tMax_) tMax_ = leftSteps_[i].tMax();

        for (size_t i = 0; i < rightSteps_.size(); ++i)
            if (rightSteps_[i].tMax() > tMax_) tMax_ = rightSteps_[i].tMax();

        int nTimeSteps = tMax_ / T_;
        Eigen::VectorXd time(nTimeSteps);
        for (long i = 0; i < nTimeSteps; i++)
        {
            time(i) = i * T_;
        }

        xMin_.resize(nTimeSteps);
        xMax_.resize(nTimeSteps);
        xMin_.setZero();
        xMax_.setZero();
        yMin_.resize(nTimeSteps);
        yMax_.resize(nTimeSteps);
        yMin_.setZero();
        yMax_.setZero();
        z_.resize(nTimeSteps);
        z_.setZero();

        for (long iTimeStep = 0; iTimeStep < nTimeSteps; iTimeStep++)
        {
            bool singleSupportLeft = false;
            bool singleSupportRight = false;
            bool doubleSupport = false;
            double currentTime = time(iTimeStep);

            size_t iStepLeft = 0;
            size_t iStepRight = 0;

            for (size_t i = 0; i < leftSteps_.size(); i++)
            {
                if (leftSteps_[i].tMin() <= currentTime && currentTime < leftSteps_[i].tMax())
                {
                    singleSupportLeft = true;
                    iStepLeft = i;
                }
            }

            for (size_t i = 0; i < rightSteps_.size(); i++)
            {
                if (rightSteps_[i].tMin() <= currentTime && currentTime < rightSteps_[i].tMax())
                {
                    singleSupportRight = true;
                    iStepRight = i;
                }
            }

            if (singleSupportLeft && singleSupportRight)
            {
                singleSupportLeft = false;
                singleSupportRight = false;
                doubleSupport = true;
            }

            if (singleSupportLeft)
            {
                xMin_[iTimeStep] = leftSteps_[iStepLeft].x() - stepXWidth_ / 2;
                xMax_[iTimeStep] = leftSteps_[iStepLeft].x() + stepXWidth_ / 2;
                yMin_[iTimeStep] = leftSteps_[iStepLeft].y() - stepYWidth_ / 2;
                yMax_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
                z_[iTimeStep] = leftSteps_[iStepLeft].z();
            }
            else if (singleSupportRight)
            {
                xMin_[iTimeStep] = rightSteps_[iStepRight].x() - stepXWidth_ / 2;
                xMax_[iTimeStep] = rightSteps_[iStepRight].x() + stepXWidth_ / 2;
                yMin_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
                yMax_[iTimeStep] = rightSteps_[iStepRight].y() + stepYWidth_ / 2;
                z_[iTimeStep] = rightSteps_[iStepRight].z();
            }
            else if (doubleSupport)
            {
                double minX = rightSteps_[iStepRight].x();
                double maxX = leftSteps_[iStepLeft].x();
                if (minX > maxX)
                {
                    double tmp = maxX;
                    maxX = minX;
                    minX = tmp;
                }
                xMin_[iTimeStep] = minX - stepXWidth_ / 2;
                xMax_[iTimeStep] = maxX + stepXWidth_ / 2;
                yMin_[iTimeStep] = rightSteps_[iStepRight].y() - stepYWidth_ / 2;
                yMax_[iTimeStep] = leftSteps_[iStepLeft].y() + stepYWidth_ / 2;
                z_[iTimeStep] = rightSteps_[iStepRight].z();
            }
        }
    }

    /// @brief Computes the plan for steps contained in the fields leftSteps_ and rightSteps_
    void computePlan() { computePlan(leftSteps_, rightSteps_); }

    /// @brief Getter for xMin
    const Eigen::VectorXd& xMin() const { return xMin_; }
    /// @brief Getter for xMax
    const Eigen::VectorXd& xMax() const { return xMax_; }
    /// @brief Getter for yMin
    const Eigen::VectorXd& yMin() const { return yMin_; }
    /// @brief Getter for yMax
    const Eigen::VectorXd& yMax() const { return yMax_; }
    /// @brief Getter for z
    const Eigen::VectorXd& z() const { return z_; }
    /// @brief Getter for tMax
    const double& tMax() const { return tMax_; }

  private:
    /// @brief List of left foot steps
    std::vector<Step> leftSteps_;
    /// @brief List of right foot steps
    std::vector<Step> rightSteps_;
    Eigen::VectorXd xMin_;
    Eigen::VectorXd xMax_;
    Eigen::VectorXd yMin_;
    Eigen::VectorXd yMax_;
    Eigen::VectorXd z_;
    double tMax_;
    double T_;
    /// @brief Width of foot along X
    double stepXWidth_;
    /// @brief Width of foot along Y
    double stepYWidth_;
    /// @brief Width of foot along Z
    double stepZWidth_;
};
} /* example */

} /* mpc */
