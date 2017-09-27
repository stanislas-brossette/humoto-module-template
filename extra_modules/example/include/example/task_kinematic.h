/**
    @file
    @author  Stanislas Brossette
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once
#include <cmath>

namespace humoto
{
namespace example
{
/// @brief Task requiring the CoM to be inside of a polygon approximating the kinematic reachable area for a biped robot
/// in the form of { Ax < ub }
class HUMOTO_LOCAL TaskKinematics : public humoto::TaskAU
{
  protected:
#define HUMOTO_CONFIG_ENTRIES HUMOTO_CONFIG_PARENT_CLASS(TaskAU)
#include HUMOTO_CONFIG_DEFINE_ACCESSORS

    /// @brief Sets the defaults
    void setDefaults()
    {
        TaskAU::setDefaults();
        number_of_vertices_ = 6;
        min_height_ = 0.6;
        max_height_ = 1.0;
        width_x_ = 0.6;
        width_y_ = 0.4;
        setGain(100);
    }

    void computePolygon(std::vector<Eigen::Vector2d> &vertices, size_t n, double min_height, double max_height,
                        double width)
    {
        vertices.push_back(Eigen::Vector2d(width/2.0, min_height);
        vertices.push_back(Eigen::Vector2d(-width/2.0, min_height);
        for (size_t i = 0; i < n-2; ++i)
        {
            Eigen::Vector2d newPoint(-width / 2.0 * cos(i * humoto::g_pi / (n - 1)),
                                     min_height + width / 2.0 * sin(i * humoto::g_pi / (n - 1)));
            vertices.push_back(newPoint);
        }
        std::cout << "Polygon:" << std::endl;
        for (size_t i = 0; i < vertices.size(); i++)
        {
            std::cout << "vertices[i]:\n" << vertices[i].transpose() << std::endl;
        }
    }

    void computePolygons()
    {
        std::cout << "Polygon XZ" << std::endl;
        computePolygon(boundary_vertices_xz_, number_of_vertices_, min_height_, max_height_, width_x_);
        std::cout << "Polygon YZ" << std::endl;
        computePolygon(boundary_vertices_yz_, number_of_vertices_, min_height_, max_height_, width_y_);
    }

    void computeC()
    {
        for (size_t i = 0; i < number_of_vertices_; i++)
        {
            size_t i0 = i;
            size_t i1 = i + 1;
            if (i1 >= number_of_vertices_) i1 = 0;

            C_.row(i) << -(boundary_vertices_xz_[i1](1) - boundary_vertices_xz_[i0](1)), 0,
                -(boundary_vertices_xz_[i1](0) - boundary_vertices_xz_[i0](0));
            C_.row(number_of_vertices_ + i) << 0, boundary_vertices_yz_[i1](1) - boundary_vertices_yz_[i0](1),
                boundary_vertices_yz_[i1](0) - boundary_vertices_yz_[i0](0);
        }
        std::cout << "C_:\n" << C_ << std::endl;
    }

    /// @brief Finalizes the class initialization
    void finalize()
    {
        computePolygons();
        C_.resize(2 * number_of_vertices_, 3);
        C_.setZero();
        computeC();
        TaskAU::finalize();
    }

  private:
    /// @brief number of vertices in each 2d polygon
    size_t number_of_vertices_;
    /// @brief min and max heights of the polygons
    double min_height_, max_height_;
    /// @brief width along x and y of the polygons
    double width_x_, width_y_;
    /// @brief list of points of the polygon in the referencial of the support foot
    std::vector<Eigen::Vector2d> boundary_vertices_xz_;
    std::vector<Eigen::Vector2d> boundary_vertices_yz_;
    Eigen::MatrixXd C_;

  public:
    /// @brief Default constructor
    ///
    /// @param gain gain of the task
    TaskKinematics(const double gain = 100) : TaskAU("TaskKinematics", gain) {}

    /// @brief Forms the matrices A and b to represent the task
    ///
    /// @param sol_structure structure of the problems solution
    /// @param model_base model (can be downcasted dynamically to a specific model type if necessary)
    /// @param control_problem control_problem (can be downcasted dynamically to a specific problem type if necessary)
    void form(const humoto::SolutionStructure &sol_structure, const humoto::Model &model_base,
              const humoto::ControlProblem &control_problem)
    {
        // Downcast the control problem into a simpleMPC type
        const humoto::example::MPCVerticalMotion &mpc =
            dynamic_cast<const humoto::example::MPCVerticalMotion &>(control_problem);

        // Initialize the matrices A and b
        Eigen::MatrixXd &A = getA();
        Eigen::VectorXd &ub = getUpperBound();

        etools::SelectionMatrix posSelector(0, 3);

        // Eigen::VectorXd cHeightRef;
        // if (cHeightRef.size() != (long)mpc.getPreviewHorizonLength())
        // cHeightRef.resize(mpc.getPreviewHorizonLength());  for (std::size_t i = 0; i < mpc.getPreviewHorizonLength();
        // ++i)
        //{
        //    cHeightRef(i) = mpc.stepPlan().z()(mpc.currentStepIndex() + i) + mpc.pbParams().comHeightRef_;
        //}

        // etools::SelectionMatrix posZselector(9, 6);
        //// Compute the A and b matrices
        // A.noalias() = getGain() * (posZselector * mpc.Uu());
        // b.noalias() = getGain() * (cHeightRef - posZselector * mpc.Ux() * mpc.currentState());
    };
};
}
}
