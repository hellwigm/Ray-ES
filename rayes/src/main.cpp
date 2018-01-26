#include "es/rayes/RayEs.h"
#include "es/rayes/Info.h"
#include "es/core/util.h"

#include <Eigen/Dense>

#include <iostream>
#include <cstdlib>

int main() {
    Eigen::MatrixXd A(1, 2);
    A << -1, 1;
    Eigen::VectorXd b(1);
    b << -5;
    auto fitnessFun = [](const Eigen::VectorXd &x) {
        return x.transpose() * x;
    };

    Eigen::VectorXd lbnds =
        -5 * Eigen::MatrixXd::Ones(A.cols(), 1);
    Eigen::VectorXd ubnds =
        5 * Eigen::MatrixXd::Ones(A.cols(), 1);

    auto constraintFun = [A, b](const Eigen::VectorXd &x) -> Eigen::VectorXd {
        return A * x - b;
    };

    const double fBest = 12.5;

    // use a feasible origin
    Eigen::VectorXd originInit = Eigen::VectorXd::Zero(lbnds.rows());
    originInit(0) = 5;
    originInit(1) = -1;
    es::rayes::RayEs
        solver(fitnessFun, constraintFun, lbnds, ubnds,
               originInit,
               es::rayes::LineSearchAlg::Modified);
    es::rayes::Info info = solver.run();
    std::cout << "Termination criterion: "
              << es::core::toString(info.getTerminationCriterion())
              << "."
              << std::endl;
    if (std::abs(fBest) < 1e-6) {
        std::cout << "abs(fBest - (f of best point)) = "
                  << std::abs(fBest - info.getBestIndividual().f())
                  << std::endl;
    } else {
        std::cout << "abs(fBest - (f of best point)) / fBest = "
                  << std::abs(fBest - info.getBestIndividual().f()) / fBest
                  << std::endl;
    }

    return EXIT_SUCCESS;
}

