#include "es/core/util.h"

#include <chrono>
#include <random>

namespace es {
namespace core {

Eigen::MatrixXd randn(int nRows, int nCols) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    std::normal_distribution<double> distribution (0.0, 1.0);

    Eigen::MatrixXd m(nRows, nCols);
    for (int row = 0; row < nRows; ++row) {
        for (int col = 0; col < nCols; ++col) {
            m(row, col) = distribution(generator);
        }
    }
    return m;
}

Eigen::MatrixXd rand(int nRows, int nCols, double lo, double hi) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    std::uniform_real_distribution<double> distribution (lo, hi);

    Eigen::MatrixXd m(nRows, nCols);
    for (int row = 0; row < nRows; ++row) {
        for (int col = 0; col < nCols; ++col) {
            m(row, col) = distribution(generator);
        }
    }
    return m;
}

}
}

