/*! \file
 *  \brief Classes for information of a run of an evolutionary strategy.
 */

#ifndef ES_RAYES_INFO_H
#define ES_RAYES_INFO_H

#include "es/rayes/Individual.h"

#include <fstream>

namespace es {
namespace rayes {

/*! \brief Reason for termination of an evolutionary strategy run.
 */
enum class TerminationCriterion {
    Unknown,
    GenerationLimitReached,
    SigmaLimitReached,
    BestSoFarNotUpdatedTooLong
};

std::ostream &operator<<(std::ostream &os,
                         TerminationCriterion terminationCriterion);

/*! \brief Encapsulates information about a run of an evolution strategy.
 */
class Info {
 public:
    Info();

    TerminationCriterion getTerminationCriterion() const;
    void setTerminationCriterion(TerminationCriterion
                                 terminationCriterion);

    int getNumFitnessEvaluations() const;
    void setNumFitnessEvaluations(int numFitnessEvaluations);

    int getNumGenerations() const;
    void setNumGenerations(int numGenerations);

    Individual getBestIndividual() const;
    void setBestIndividual(const Individual &bestIndividual);

 private:
    TerminationCriterion m_terminationCriterion;
    int m_numFitnessEvaluations;
    int m_numGenerations;
    Individual m_bestIndividual;
};

}
}

#endif
