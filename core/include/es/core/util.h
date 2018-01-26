/*! \file
 *  \brief Contains utilities.
 */

#ifndef ES_CORE_UTIL_H
#define ES_CORE_UTIL_H

#include <string>
#include <sstream>

#include <Eigen/Dense>

namespace es {
namespace core {

/*! A constant for infinity. */
constexpr double INFTY = 1e10;

/*!
 * \brief Converts the given value to a string.
 *
 * Converts the given value to a string using operator<<.
 */
template<typename T>
std::string toString(const T &v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

/*!
 * \brief Initializes a matrix with
 * iid standard normally distributed random variates.
 *
 * Initializes a matrix with iid normally distributed random variates.
 * The size of the matrix is nRows times nCols.
 */
Eigen::MatrixXd randn(int nRows, int nCols);

/*!
* \brief Initializes a matrix with iid uniformly distributed random variates.
*
* Initializes a matrix with iid uniformly distributed random variates.
* The size of the matrix is nRows times nCols.
* The bounds are lo and hi.
*/
Eigen::MatrixXd rand(int nRows, int nCols, double lo, double hi);

}
}

#endif
