#include <Eigen/Eigen>

#include <drake/common/symbolic/expression.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/decision_variable.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <iostream>

namespace manr {
namespace learn {

using drake::solvers::Binding;
using drake::solvers::LinearCost;
using drake::solvers::MathematicalProgram;
using drake::solvers::QuadraticCost;
using drake::symbolic::Variable;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void RunMProg1() {
  auto var1 = Variable("x");
  auto var2 = Variable("u");
  std::cout << var1.get_id() << std::endl;
  std::cout << var2.get_id() << std::endl;

  auto linear_program = MathematicalProgram();
  auto w = linear_program.NewContinuousVariables(25, 1, "W");
  auto linear_cost = std::make_shared<LinearCost>(VectorXd::Ones(25), 7.);
  Binding linear_binding{linear_cost, w};
  linear_program.AddCost(linear_binding);

  auto quadratic_program = MathematicalProgram();
  auto u = quadratic_program.NewContinuousVariables(3, 1, "U");
  MatrixXd q(3, 3);
  q << 0.55, -0.6, 0.3, 0.9, 7.6, -0.34, -4.5, 9.06, 0.1;
  auto quadratic_cost = std::make_shared<QuadraticCost>(q, VectorXd::Zero(3));
  std::cout << "Quadratic cost is convex? " << quadratic_cost->is_convex() << std::endl;
  Binding quadratic_binding{quadratic_cost, u};
  quadratic_program.AddCost(quadratic_binding);

  // auto custom_program = MathematicalProgram();
  // auto x = custom_program.NewContinuousVariables(25, 3, "X");
  // auto u = custom_program.NewContinuousVariables(25, 2, "U");

  auto quadratic_result = drake::solvers::Solve(quadratic_program);
  std::cout << "Quadratic results: " << std::endl;
  std::cout << "=============" << std::endl;
  std::cout << "Status: " << quadratic_result.get_solution_result() << std::endl;
  std::cout << "Optimal cost: " << quadratic_result.get_optimal_cost() << std::endl;
  std::cout << "Optimal solution: " << quadratic_result.GetSolution().transpose() << std::endl;
  std::cout << "=============" << std::endl;
}

} // namespace learn
} // namespace manr

int main(int argc, char *argv[]) { manr::learn::RunMProg1(); }
