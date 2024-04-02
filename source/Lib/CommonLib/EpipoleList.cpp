//
// Created by regensky on 8/16/22.
//

#include "EpipoleList.h"
#include <iostream>

void EpipoleList::addEpipole(const Array3TCoord &epipole, const int curPOC, const int refPOC, bool makeAvailable) {
  auto epipoleSpherical = FloatingFixedConversion::floatingToFixed(CoordinateConversion::cartesianToSpherical(epipole), EPIPOLE_PRECISION_FIXED);
  EpipoleEntry entry({epipoleSpherical.coeff(1), epipoleSpherical.coeff(2)}, makeAvailable);
  m_epipoleMap[{curPOC, refPOC}] = entry;
}

Array3TCoord EpipoleList::findEpipole(int curPOC, int refPOC) const
{
  const auto epipoleFixedSpherical = findEpipoleFixed(curPOC, refPOC);
  const auto epipoleSpherical = FloatingFixedConversion::fixedToFloating(epipoleFixedSpherical, EPIPOLE_PRECISION_FIXED);
  return CoordinateConversion::sphericalToCartesian({1, epipoleSpherical.coeff(0), epipoleSpherical.coeff(1)});
}

Array2Fixed EpipoleList::findEpipoleFixed(int curPOC, int refPOC) const
{
  // Check if explicit entry for given (curPOC, refPOC) combination exists
  auto iter = m_epipoleMap.find({curPOC, refPOC});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  // Check if per POC entry for given curPOC exist
  iter = m_epipoleMap.find({curPOC, -1});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  // Check if global entry exists
  iter = m_epipoleMap.find({-1, -1});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return iter->second.epipole;
  }

  CHECK(true, "No epipole for given (curPOC, refPOC) combination (" + std::to_string(curPOC) + ", " + std::to_string(refPOC) + ") found.");
}

Array2TCoord EpipoleList::derivePredictor(int curPOC) const
{
  switch (m_predictionMode)
  {
  case PredictionMode::NONE:
    return {0, 0};
  case PredictionMode::CLOSEST:
    return derivePredictorClosest(curPOC);
  case PredictionMode::CLOSEST_OLD:
    return derivePredictorClosest(curPOC, true);
  default:
    CHECK(true, "Unknown prediction mode.")
  }
}

Array2TCoord EpipoleList::derivePredictorClosest(int curPOC, bool old) const
{
  const auto globalEpipoleEntry = m_epipoleMap.at({-1, -1});
  CHECK(!globalEpipoleEntry.isAvailable, "Global epipole is not available.");

  int minPOCDistances[2] = { std::numeric_limits<int>::max(), std::numeric_limits<int>::max() };
  Array2Fixed predictors[2] = { globalEpipoleEntry.epipole, globalEpipoleEntry.epipole };
  for (const auto &item : m_epipoleMap)
  {
    if (!item.second.isAvailable)
    {
      continue;
    }
    int distance = abs(curPOC - item.first.first);
    if (distance < minPOCDistances[0])
    {
      minPOCDistances[0] = distance;
      predictors[0] = item.second.epipole;
    }
    else if (distance < minPOCDistances[1])
    {
      minPOCDistances[1] = distance;
      predictors[1] = item.second.epipole;
    }
  }

  Array2Fixed predictor;
  if (minPOCDistances[0] == minPOCDistances[1])
  {
    if (old) {
      predictor = predictors[0] + predictors[1] / 2;
    } else {
      predictor = (predictors[0] + predictors[1]) / 2;
    }
  }
  else
  {
    CHECK(minPOCDistances[0] > minPOCDistances[1], "Smallest POC distance is larger than second smallest POC distance.");
    predictor = predictors[0];
  }

  return FloatingFixedConversion::fixedToFloating(predictor, EPIPOLE_PRECISION_FIXED);
}

bool EpipoleList::hasEpipole(int curPOC, int refPOC) const
{
  const auto iter = m_epipoleMap.find({curPOC, refPOC});
  if (iter != m_epipoleMap.end() && iter->second.isAvailable) {
    return true;
  }
  return false;
}

void EpipoleList::makeAvailable(int curPOC)
{
  for (auto& iter : m_epipoleMap)
  {
    if (iter.first.first == curPOC) {
      iter.second.isAvailable = true;
    }
  }
}

void EpipoleList::printSummary() const
{
  std::cout << "\n\n----- Epipole config -----\n";
  for (auto &iter: m_epipoleMap) {
    if (iter.first.first == -1 && iter.first.second == -1 && iter.second.epipole.isZero()) {
      continue;
    }
    const auto epipoleSpherical = FloatingFixedConversion::fixedToFloating(iter.second.epipole, EPIPOLE_PRECISION_FIXED);
    const auto epipole = CoordinateConversion::sphericalToCartesian({1, epipoleSpherical.coeff(0), epipoleSpherical.coeff(1)});
    std::cout << iter.first.first << ", " << iter.first.second << ": (" << epipole << ")\n";
  }
  std::cout << "----- Epipole config -----\n";
}

