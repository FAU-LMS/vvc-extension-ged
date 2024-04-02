//
// Created by regensky on 7/20/22.
//

#ifndef VTM360_MMCONFIG_H
#define VTM360_MMCONFIG_H

#include <set>
#include "CommonDef.h"
#include "Coordinate.h"
#include "MotionModels/GeodesicMotionModel.h"

struct MMConfig
{
  bool              GED{false}; /**< Geodesic motion model */
  GeodesicMotionModel::Flavor GEDFlavor{GeodesicMotionModel::VISHWANATH_ORIGINAL}; /**< Geodesic motion model flavor for geodesic motion models */
  bool              MMMVP{false}; /**< Multi-model motion vector prediction */
  int               MMOffset4x4{0}; /**< Multi-model 4x4 subblock offset */
  int               projectionFct{0}; /**< Projection function */
  Array3Fixed       globalEpipole{0,0,0};

  bool getUseMultiModel() const { return GED; }
  std::vector<MotionModelID> getActiveMotionModels() const;
};

#endif   // VTM360_MMCONFIG_H
