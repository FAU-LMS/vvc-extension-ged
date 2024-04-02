//
// Created by regensky on 7/20/22.
//

#include "MMConfig.h"

std::vector<MotionModelID> MMConfig::getActiveMotionModels() const {
  {
    std::vector<MotionModelID> activeMotionModels{ CLASSIC };
    if (GED)
    {
      activeMotionModels.push_back(GEODESIC);
    }
    return activeMotionModels;
  }
}
