#include "es/rayes/RayEs.h"
#include "es/rayes/Individual.h"

#include "es/core/util.h"

#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

namespace es {
namespace rayes {

RayEs::RayEs(
       const std::function<double(const Eigen::VectorXd &)> &objectiveFun,
       const std::function<Eigen::VectorXd(
                          const Eigen::VectorXd &)> &constraintFun,
       const Eigen::VectorXd &lbnds,
       const Eigen::VectorXd &ubnds,
       const Eigen::VectorXd &rayOriginInit,
       const LineSearchAlg lineSearchAlg)
    : m_objectiveFun(objectiveFun)
    , m_constraintFun(constraintFun)
    , m_lbnds(lbnds)
    , m_ubnds(ubnds)
    , m_rayOriginInit(rayOriginInit)
    , m_lineSearchAlg(lineSearchAlg)
{
}

Info RayEs::run() {
    Info info;

    const int dimension = m_lbnds.rows();
    assert(dimension == m_ubnds.rows());

    const int lambda = 4 * dimension;
    const int mu = lambda / 4;
    const double sigmaInit = 1.0 / sqrt(static_cast<double>(dimension));
    const double tau = 1 / sqrt(2.0 * static_cast<double>(dimension));
    const int gLag = 50 * dimension;
    const int gStop = 100000;
    const double sigmaStop = 1e-6;
    const double lineSearchLineLength =
        2.0 * std::abs(m_ubnds.maxCoeff() - m_lbnds.minCoeff());
    const int lineSearchPartitions = 2;
    const double lineSearchEpsilon = 1e-10;
    const double lineSearchStepSizeIncreaseFactor = 1.5;
    const double lineSearchStepSizeDecreaseFactor = 10.0;
    if (!(lambda >= mu)) {
        throw std::runtime_error("lambda must be greater or equal to mu");
    }

    info.setNumFitnessEvaluations(0);

    auto fEvalHelper = [&info, this](const Eigen::VectorXd &x) {
        info.setNumFitnessEvaluations(info.getNumFitnessEvaluations() + 1);
        return m_objectiveFun(x);
    };

    auto isFirstFitterThanSecond = [](const Individual &a,
                                      const Individual &b) {
        return a.f() < b.f();
    };

    double sigma = sigmaInit;
    double sigmaRayOrigin = sigmaInit;
    Eigen::VectorXd rayInit = es::core::randn(dimension, 1);
    const double rayInitNorm = rayInit.norm();
    if (std::abs(rayInitNorm) > 1e-6) {
        rayInit /= std::abs(rayInitNorm);
    }
    Eigen::VectorXd ray = rayInit;
    Eigen::VectorXd rayOrigin = m_rayOriginInit;
    Eigen::VectorXd bestOnRayPrev = rayOrigin;

    Individual a;
    if (isFeasible(rayOrigin)) {
        a.f(fEvalHelper(rayOrigin));
    } else {
        a.f(std::numeric_limits<double>::max());
    }
    a.ray(rayOrigin);
    a.bestOnRay(rayOrigin);
    a.sigma(sigma);
    a.rayOrigin(rayOrigin);
    a.sigmaRayOrigin(sigmaRayOrigin);

    Individual aBest = a;
    int aBestG = 0;

    std::set<int> bestDirections({-1, 1});

    // create a wrapper function for the line search depending on the type
    // configured in the constructor
    auto lineSearchFunction =
        [&]() -> std::function<LineSearchResult(const Individual &individual)> {
        if (m_lineSearchAlg == LineSearchAlg::Standard) {
            return [&](const Individual &individual) -> LineSearchResult {
                return lineSearch(individual.rayNormalized(),
                                  lineSearchLineLength,
                                  lineSearchPartitions,
                                  m_rayOriginInit,
                                  lineSearchEpsilon);
            };
        } else if (m_lineSearchAlg == LineSearchAlg::Modified) {
            return [&](const Individual &individual) -> LineSearchResult {
                LineSearchResult lineSearchResult;
                // yields 1 for ray in same direction
                // and 0 for ray in orthogonal direction
                const double similarityByDotProduct =
                    std::abs(individual.ray().dot(ray));
                // if parent and offspring have almost the same direction...
                if (std::abs(similarityByDotProduct - 1.0) < 1e-3) {
                    // ... take the parental bestOnRay point as a good initial
                    // guess and search around this one
                    // for this, project the parental bestOnRay point onto
                    // the current ray and compute the step size and a new
                    // origin
                    Eigen::VectorXd bestOnRayPrevProjectedOntoOffspringRay =
                        m_rayOriginInit +
                        individual.ray() *
                        (bestOnRayPrev -
                         m_rayOriginInit).dot(individual.ray());
                    Eigen::VectorXd originToUse = m_rayOriginInit;
                    if (bestDirections.size() == 1) {
                        // origin is slightly in opposite direction of search
                        // direction to include possible improvements
                        // missed in previous iterations due to the step size
                        const int d = *bestDirections.begin();
                        originToUse =
                            bestOnRayPrevProjectedOntoOffspringRay -
                            d * 1e-1 * individual.ray();
                    }
                    const double stepSizeGuess =
                        (bestOnRayPrevProjectedOntoOffspringRay -
                         originToUse)
                        .norm() / (individual.ray().norm());
                    lineSearchResult =
                        lineSearch2(individual.rayNormalized(),
                                    originToUse,
                                    stepSizeGuess,
                                    lineSearchStepSizeIncreaseFactor,
                                    lineSearchStepSizeDecreaseFactor,
                                    lineSearchEpsilon,
                                    bestDirections);
                } else {
                    lineSearchResult =
                    lineSearch2(individual.rayNormalized(),
                                m_rayOriginInit,
                                lineSearchLineLength,
                                lineSearchStepSizeIncreaseFactor,
                                lineSearchStepSizeDecreaseFactor,
                                lineSearchEpsilon,
                                bestDirections);
                }
                return lineSearchResult;
            };
        } else {
            throw std::runtime_error("unknown line search algorithm type");
        }
    }();

    int g = 0;
    do {
        // std::cout << "g: " << g << "\n";
        // std::cout << "sigma: " << sigma << "\n";
        // std::cout << "f: " << aBest.f() << "\n";

        std::vector<Individual> offspring;
        for (int k = 0; k < lambda; ++k) {
            Individual currOffspring;
            currOffspring.sigma(sigma *
                                std::exp(tau * es::core::randn(1, 1)(0)));
            currOffspring.ray(ray + currOffspring.sigma() *
                              es::core::randn(dimension, 1));
            currOffspring.ray(currOffspring.rayNormalized());
            LineSearchResult lineSearchResult =
                lineSearchFunction(currOffspring);
            info.setNumFitnessEvaluations(info.getNumFitnessEvaluations() +
                                      lineSearchResult.numFitnessEvaluations);
            currOffspring.bestOnRay(lineSearchResult.bestOnRay);
            currOffspring
                .bestOnRayDirection(lineSearchResult.bestOnRayDirection);
            currOffspring.f(lineSearchResult.f);
            if (lineSearchResult.feasibleFound) {
                if (isFirstFitterThanSecond(currOffspring, aBest)) {
                    aBest = currOffspring;
                    aBestG = g + 1;
                }
                offspring.push_back(currOffspring);
            }
        }

        std::sort(offspring.begin(), offspring.end(), isFirstFitterThanSecond);

        int nFeasible = static_cast<int>(offspring.size());
        int div = std::min(mu, nFeasible);
        if (nFeasible > 0) {
            double weight = 1.0 / static_cast<double>(div);
            Eigen::VectorXd rayCentroid = Eigen::MatrixXd::Zero(dimension, 1);
            double sigmaCentroid = 0;
            Eigen::VectorXd bestOnRayCentroid =
                Eigen::MatrixXd::Zero(dimension, 1);
            if (m_lineSearchAlg == LineSearchAlg::Modified) {
                bestDirections.clear();
            }
            for (int k = 0; k < div; ++k) {
                const auto &currOffspring = offspring.at(k);
                rayCentroid = rayCentroid + weight * currOffspring.ray();
                sigmaCentroid = sigmaCentroid + weight * currOffspring.sigma();
                bestOnRayCentroid += weight * currOffspring.bestOnRay();
                if (m_lineSearchAlg == LineSearchAlg::Modified) {
                    const int d = currOffspring.bestOnRayDirection();
                    if (!(1 == d || -1 == d)) {
                        throw std::runtime_error("unexpected direction");
                    }
                    bestDirections.insert(d);
                }
            }
            bestOnRayPrev = bestOnRayCentroid;
            ray = rayCentroid;
            double norm = ray.norm();
            if (std::abs(norm) > 1e-6) {
                ray /= std::abs(norm);
            }
            sigma = sigmaCentroid;
        }

        g += 1;
    } while (!(sigma < sigmaStop || g > gStop || g - aBestG >= gLag));

    if (g > gStop) {
        info.setTerminationCriterion(
                        TerminationCriterion::GenerationLimitReached);
    } else if (sigma < sigmaStop) {
        info.setTerminationCriterion(TerminationCriterion::SigmaLimitReached);
    } else if (g - aBestG >= gLag) {
        info.setTerminationCriterion(
                     TerminationCriterion::BestSoFarNotUpdatedTooLong);
    }

    info.setNumGenerations(g);
    info.setBestIndividual(aBest);

    return info;
}

LineSearchResult RayEs::lineSearch(const Eigen::VectorXd &rayNormalized,
                                   const double initialLength,
                                   const int nPartitions,
                                   const Eigen::VectorXd &rayOrigin,
                                   const double epsilon) {
    const int dimension = m_lbnds.rows();
    LineSearchResult lineSearchResult;
    auto fEvalHelper = [&lineSearchResult, this](const Eigen::VectorXd &x) {
        lineSearchResult.numFitnessEvaluations += 1;
        return m_objectiveFun(x);
    };
    lineSearchResult.feasibleFound = false;
    Eigen::VectorXd rayOriginCurr = rayOrigin;
    double rayOriginCurrFitness = fEvalHelper(rayOriginCurr);
    double deltaPartition = initialLength / static_cast<double>(nPartitions);
    while (deltaPartition > epsilon) {
        int nFeasible = 0;
        double bestFitness = std::numeric_limits<double>::max();
        Eigen::VectorXd bestPos = Eigen::VectorXd::Zero(dimension);
        for (int p = 1; p <= 2 * nPartitions + 1; ++p) {
            Eigen::VectorXd pos = rayOriginCurr +
                deltaPartition *
                static_cast<double>((p - nPartitions - 1)) * rayNormalized;
            if (isFeasible(pos)) {
                ++nFeasible;
                double fitness = fEvalHelper(pos);
                if (fitness < bestFitness) {
                    bestFitness = fitness;
                    bestPos = pos;
                }
            }
        }
        if (nFeasible > 0) {
            rayOriginCurr = bestPos;
            rayOriginCurrFitness = bestFitness;
            lineSearchResult.feasibleFound = true;
        }

        deltaPartition = deltaPartition / static_cast<double>(nPartitions);
    }

    lineSearchResult.bestOnRay = rayOriginCurr;
    lineSearchResult.f = rayOriginCurrFitness;

    return lineSearchResult;
}

LineSearchResult RayEs::lineSearch2(const Eigen::VectorXd &rayNormalized,
                                    const Eigen::VectorXd &rayOrigin,
                                    const double stepSizeInit,
                                    const double stepSizeIncreaseFactor,
                                    const double stepSizeDecreaseFactor,
                                    const double epsilon,
                                    const std::set<int> &directions) {
    const int dimension = m_lbnds.rows();
    LineSearchResult lineSearchResult;
    auto fEvalHelper = [&lineSearchResult, this](const Eigen::VectorXd &x) {
        lineSearchResult.numFitnessEvaluations += 1;
        return m_objectiveFun(x);
    };
    lineSearchResult.feasibleFound = false;
    lineSearchResult.bestOnRay = Eigen::VectorXd::Zero(dimension);
    lineSearchResult.f = std::numeric_limits<double>::max();
    lineSearchResult.bestOnRayDirection = 0;
    const double rayOriginFitness = fEvalHelper(rayOrigin);
    const bool isRayOriginFeasible = isFeasible(rayOrigin);
    for (int direction : directions) {
        Eigen::VectorXd rayOriginCurr = rayOrigin;
        double rayOriginCurrFitness = rayOriginFitness;
        bool isRayOriginCurrFeasible = isRayOriginFeasible;
        Eigen::VectorXd rayOriginPrev = rayOriginCurr;
        double rayOriginPrevFitness = rayOriginCurrFitness;
        double stepSize = stepSizeInit;
        int iter = 0;
        while (iter < 100 && stepSize >= epsilon) {
            rayOriginCurr = rayOriginPrev;
            rayOriginCurrFitness = rayOriginPrevFitness;
            do {
                rayOriginPrev = rayOriginCurr;
                rayOriginPrevFitness = rayOriginCurrFitness;

                rayOriginCurr = rayOriginCurr +
                    static_cast<double>(direction) * stepSize *
                    rayNormalized;
                isRayOriginCurrFeasible = isFeasible(rayOriginCurr);
                if (isRayOriginCurrFeasible) {
                    rayOriginCurrFitness = fEvalHelper(rayOriginCurr);
                }

                if (isRayOriginCurrFeasible &&
                        rayOriginCurrFitness < lineSearchResult.f) {
                    stepSize *= stepSizeIncreaseFactor;
                    lineSearchResult.feasibleFound = true;
                    lineSearchResult.bestOnRay = rayOriginCurr;
                    lineSearchResult.f = rayOriginCurrFitness;
                    lineSearchResult.bestOnRayDirection = direction;
                } else {
                    stepSize /= stepSizeDecreaseFactor;
                }
            } while (isRayOriginCurrFeasible &&
                     rayOriginCurrFitness < rayOriginPrevFitness &&
                     std::abs(rayOriginCurrFitness -
                              rayOriginPrevFitness) > 1e-10);
            ++iter;
        }
    }

    return lineSearchResult;
}

bool RayEs::isFeasible(const Eigen::VectorXd &x) {
    return ((m_constraintFun(x).array() <= 1e-20).all() &&
            ((m_lbnds - x).array() <= 1e-20).all() &&
            ((x - m_ubnds).array() <= 1e-20).all());
}

}
}
