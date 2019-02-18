#pragma once

#include <Eigen/Core>
#include <map>
#include <vector>

namespace mc_solver
{

struct RobotMsg
{
  std::map<std::string, std::vector<double>> q;
  std::map<std::string, std::vector<double>> alphaVec;
  //Eigen::VectorXd alphaDVec;
  std::map<std::string, std::vector<double>> alphaDVec;
  std::map<std::string, std::vector<double>> torque;
};

} // namespace mc_solver
