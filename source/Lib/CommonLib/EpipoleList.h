//
// Created by regensky on 8/16/22.
//

#pragma once

#include <map>
#include <utility>
#include "Coordinate.h"
#include "Unit.h"

class EpipoleList
{
public:
  enum class PredictionMode
  {
    NONE,
    CLOSEST,
    CLOSEST_OLD
  };

public:
  EpipoleList(): m_epipoleMap() {
    addEpipole({1, 0, 0});
  }

  void addEpipole(const Array3TCoord &epipole, int curPOC = -1, int refPOC = -1, bool makeAvailable = false);
  Array3TCoord findEpipole(int curPOC, int refPOC) const;
  int count() const {
    bool globalIsDefault = m_epipoleMap.at({-1, -1}).epipole.isZero();
    return int(m_epipoleMap.size()) - (globalIsDefault ? 1 : 0);
  }

  bool hasEpipole(int curPOC, int refPOC) const;

  void setPredictionMode(PredictionMode predictionMode) {
    m_predictionMode = predictionMode;
  }
  PredictionMode getPredictionMode() const { return m_predictionMode; }

  /** @brief Derive the epipole predictor for the current POC from the available epipoles. */
  Array2TCoord derivePredictor(int curPOC) const;

  /** @brief Make the epipoles for the current POC available. */
  void makeAvailable(int curPOC);

  void printSummary() const;

protected:
  typedef std::pair<int, int> POCHash;
  struct EpipoleEntry {
    Array2Fixed epipole;
    bool isAvailable;

    EpipoleEntry(): epipole(0, 0), isAvailable(false) {}
    explicit EpipoleEntry(Array2Fixed epipole, bool isAvailable = false): epipole(std::move(epipole)), isAvailable(isAvailable) {}
  };

  Array2Fixed findEpipoleFixed(int curPOC, int refPOC) const;

  Array2TCoord derivePredictorClosest(int curPOC, bool old = false) const;

  std::map<POCHash, EpipoleEntry> m_epipoleMap;
  PredictionMode                  m_predictionMode;
};
