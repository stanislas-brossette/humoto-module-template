#pragma once
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <fstream>

namespace humoto
{
  namespace example
  {
    class HUMOTO_LOCAL StateHistory
    {
      public:
        StateHistory(double timeStep)
          : timeStep_(timeStep)
        {
        }
        void addStateAndControl(const etools::Vector9 state, const etools::Vector3 control)
        {
          Eigen::Vector3d position, velocity, acceleration;
          position << state(0), state(3), state(6);
          velocity << state(1), state(4), state(7);
          acceleration << state(2), state(5), state(8);
          positionsCoM_.push_back(position);
          velocitiesCoM_.push_back(velocity);
          accelerationsCoM_.push_back(acceleration);
          jerksCoM_.push_back(control);
        }
        
        Eigen::MatrixXd getPositionsAsMatrix() const { return toMatrix(positionsCoM_); }
        Eigen::MatrixXd getVelocitiesAsMatrix() const { return toMatrix(velocitiesCoM_); }
        Eigen::MatrixXd getAccelerationsAsMatrix() const { return toMatrix(accelerationsCoM_); }
        Eigen::MatrixXd getJerksAsMatrix() const { return toMatrix(jerksCoM_); }

        Eigen::MatrixXd toMatrix(const std::vector<Eigen::Vector3d>& vec) const
        {
          Eigen::MatrixXd mat;
          mat.resize(vec.size(),3);
          for (size_t i = 0; i < vec.size(); ++i)
            mat.row(i) << vec[i].transpose();
          return mat;
        }
        size_t size() const
        {
          return positionsCoM_.size();
        }
        void print() const
        {
          std::cout << "Position History:" << std::endl;
          std::cout << toMatrix(positionsCoM_) << std::endl;
          std::cout << "Velocity History:" << std::endl;
          std::cout << toMatrix(velocitiesCoM_) << std::endl;
          std::cout << "Acceleration History:" << std::endl;
          std::cout << toMatrix(accelerationsCoM_) << std::endl;
          std::cout << "Jerk History:" << std::endl;
          std::cout << toMatrix(jerksCoM_) << std::endl;
        }
        
        void plot() const
        {
          Eigen::IOFormat cleanFmt(4, 0, ", ", "\n", "[", "]");
          std::ofstream logFile("plotFile.py");
          logFile << "import numpy as np\n"
                  << "import matplotlib.pyplot as plt\n"
                  << "t = np.arange(0.," << size()
                  << ")\n\n";

          Eigen::MatrixXd positions(toMatrix(positionsCoM_));
          Eigen::MatrixXd velocities(toMatrix(velocitiesCoM_));
          Eigen::MatrixXd accelerations(toMatrix(accelerationsCoM_));
          Eigen::MatrixXd jerks(toMatrix(jerksCoM_));
        
          logFile << "x = np.array(" << positions.col(0).transpose().format(cleanFmt) << ")\n";
          logFile << "y = np.array(" << positions.col(1).transpose().format(cleanFmt) << ")\n";
          logFile << "z = np.array(" << positions.col(2).transpose().format(cleanFmt) << ")\n";
          logFile << "dx = np.array(" << velocities.col(0).transpose().format(cleanFmt) << ")\n";
          logFile << "dy = np.array(" << velocities.col(1).transpose().format(cleanFmt) << ")\n";
          logFile << "dz = np.array(" << velocities.col(2).transpose().format(cleanFmt) << ")\n";
          logFile << "ddx = np.array(" << accelerations.col(0).transpose().format(cleanFmt) << ")\n";
          logFile << "ddy = np.array(" << accelerations.col(1).transpose().format(cleanFmt) << ")\n";
          logFile << "ddz = np.array(" << accelerations.col(2).transpose().format(cleanFmt) << ")\n";
          logFile << "dddx = np.array(" << jerks.col(0).transpose().format(cleanFmt) << ")\n";
          logFile << "dddy = np.array(" << jerks.col(1).transpose().format(cleanFmt) << ")\n";
          logFile << "dddz = np.array(" << jerks.col(2).transpose().format(cleanFmt) << ")\n";

          logFile << "f, (ax1, ax2, ax3, ax4) = plt.subplots(4, sharex=True, sharey=False)\n";
          logFile << "ax1.plot(t, x, 'r', label='x')\n";
          logFile << "ax1.plot(t, y, 'b', label='y')\n";
          logFile << "ax1.plot(t, z, 'g', label='z')\n";
          logFile << "ax2.plot(t, dx, 'r', label='dx')\n";
          logFile << "ax2.plot(t, dy, 'b', label='dy')\n";
          logFile << "ax2.plot(t, dz, 'g', label='dz')\n";
          logFile << "ax3.plot(t, ddx, 'r', label='ddx')\n";
          logFile << "ax3.plot(t, ddy, 'b', label='ddy')\n";
          logFile << "ax3.plot(t, ddz, 'g', label='ddz')\n";
          logFile << "ax4.plot(t, dddx, 'r', label='dddx')\n";
          logFile << "ax4.plot(t, dddy, 'b', label='dddy')\n";
          logFile << "ax4.plot(t, dddz, 'g', label='dddz')\n";
          logFile << "ax1.set_title('Positions')\n";
          logFile << "ax2.set_title('Velocitities')\n";
          logFile << "ax3.set_title('Accelerations')\n";
          logFile << "ax4.set_title('Jerks')\n";
          //logFile << "f.subplots_adjust(hspace=0)\n";
          logFile << "legend = ax1.legend(loc='upper center', shadow=True)\n";
          logFile << "legend = ax2.legend(loc='upper center', shadow=True)\n";
          logFile << "legend = ax3.legend(loc='upper center', shadow=True)\n";
          logFile << "legend = ax4.legend(loc='upper center', shadow=True)\n";
          logFile << "plt.axis('auto')\n";
          logFile << "plt.show()\n";
          logFile.close();
        } 


      private:
        std::vector<Eigen::Vector3d> positionsCoM_;
        std::vector<Eigen::Vector3d> velocitiesCoM_;
        std::vector<Eigen::Vector3d> accelerationsCoM_;
        std::vector<Eigen::Vector3d> jerksCoM_;
        double timeStep_;
    };
  } /* example */ 
} /* humoto */ 
