#include "es/rayes/Info.h"

namespace es {
namespace rayes {

std::ostream &operator<<(std::ostream &os,
                         TerminationCriterion terminationCriterion) {
    if (TerminationCriterion::Unknown== terminationCriterion) {
        os << "Unknown";
    } else if (TerminationCriterion::GenerationLimitReached ==
               terminationCriterion) {
        os << "GenerationLimitReached";
    } else if (TerminationCriterion::SigmaLimitReached ==
               terminationCriterion) {
        os << "SigmaLimitReached";
    } else if (TerminationCriterion::BestSoFarNotUpdatedTooLong ==
               terminationCriterion) {
        os << "BestSoFarNotUpdatedTooLong";
    } else {
        throw std::runtime_error("Unknown termination criterion");
    }
    return os;
}

Info::Info()
    : m_terminationCriterion()
    , m_numFitnessEvaluations(0)
    , m_numGenerations(0)
{
}

TerminationCriterion Info::getTerminationCriterion() const {
    return m_terminationCriterion;
}

void Info::setTerminationCriterion(TerminationCriterion
                                   terminatinCriterion) {
    m_terminationCriterion = terminatinCriterion;
}

int Info::getNumFitnessEvaluations() const {
    return m_numFitnessEvaluations;
}

void Info::setNumFitnessEvaluations(int numFitnessEvaluations) {
    m_numFitnessEvaluations = numFitnessEvaluations;
}

int Info::getNumGenerations() const {
    return m_numGenerations;
}

void Info::setNumGenerations(int numGenerations) {
    m_numGenerations = numGenerations;
}

Individual Info::getBestIndividual() const {
    return m_bestIndividual;
}

void Info::setBestIndividual(const Individual &bestIndividual) {
    m_bestIndividual = bestIndividual;
}

}
}
