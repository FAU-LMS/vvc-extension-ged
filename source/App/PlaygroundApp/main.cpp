//
// Created by regensky on 02.03.21.
//

#include <iostream>
#include <bitset>
#include "Coordinate.h"
#include "Projection.h"
#include "MVReprojection.h"
using namespace std;

bool compare(Array2TCoord array, ArrayXXTCoordPtrPair arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1);
}

bool compare(Array3TCoord array, ArrayXXTCoordPtrTriple arrayPtr) {
  TCoord diffX = array.x() - std::get<0>(arrayPtr)->coeff(0);
  TCoord diffY = array.y() - std::get<1>(arrayPtr)->coeff(0);
  TCoord diffZ = array.z() - std::get<2>(arrayPtr)->coeff(0);

  return (abs(diffX) < 1 && abs(diffY) < 1 && abs(diffZ) < 1);
}

void testGEDEpipoleMVDerivation() {
  Size m_resolution(2216, 1108);
  TCoord m_offset4x4 = 1;
  ArrayXXTCoordPtr cart2DProjX = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, 1, Eigen::Dynamic>::LinSpaced(m_resolution.width / 4, m_offset4x4, TCoord(m_resolution.width - 4) + m_offset4x4).replicate(m_resolution.height / 4, 1));
  ArrayXXTCoordPtr cart2DProjY  = std::make_shared<ArrayXXTCoord>(Eigen::Array<TCoord, Eigen::Dynamic, 1>::LinSpaced(m_resolution.height / 4, m_offset4x4, TCoord(m_resolution.height - 4) + m_offset4x4).replicate(1, m_resolution.width / 4));
  SPS sps;
  sps.setUseGED(true);
  sps.setMMOffset4x4(1);
  sps.setUseMMMVP(true);

  EpipoleList epipoleList;
  epipoleList.addEpipole({-0.384, 0.133, 0.742}, 2, -1, true);
  epipoleList.addEpipole({0.872, 0.038, 0.111}, 3, -1, true);

  auto* projection = new EquirectangularProjection(m_resolution);

  MVReprojection mvReprojection;
  mvReprojection.init(projection, m_resolution, &sps, &epipoleList);

  Position position(338, 203);
  Mv mvOrig(19, 11);
  mvOrig.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
  MotionModelID modelIDOrig = GEODESIC;
  MotionModelID modelIDDesired = GEODESIC;
  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL;
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL;
  int curPOCOrig = 2;
  int refPOCOrig = -1;
  int curPOCDesired = 3;
  int refPOCDesired = -1;
  Mv mvDesired = mvReprojection.motionVectorInDesiredMotionModel(position, mvOrig, modelIDOrig, modelIDDesired, shiftHor, shiftVer, curPOCOrig, refPOCOrig, curPOCDesired, refPOCDesired,
                                                                                 position, Size(1, 1), position, Size(1, 1));

  const TCoord mvOrigX = TCoord(mvOrig.hor >> shiftHor) + TCoord(mvOrig.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  const TCoord mvOrigY = TCoord(mvOrig.ver >> shiftVer) + TCoord(mvOrig.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  const Array2TCoord mvOrig_(mvOrigX, mvOrigY);
  const TCoord mvDesiredX = TCoord(mvDesired.hor >> shiftHor) + TCoord(mvDesired.hor & ((1 << shiftHor) - 1))/TCoord(1 << shiftHor);
  const TCoord mvDesiredY = TCoord(mvDesired.ver >> shiftVer) + TCoord(mvDesired.ver & ((1 << shiftVer) - 1))/TCoord(1 << shiftVer);
  const Array2TCoord mvDesired_(mvDesiredX, mvDesiredY);

  std::cout << mvOrig_ << " -> " << mvDesired_ << std::endl;

  GeodesicMotionModel* motionModel = static_cast<GeodesicMotionModel*>(mvReprojection.getMotionModel(GEODESIC));

  ArrayXXTCoordPtr cart2DX = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  ArrayXXTCoordPtr cart2DY = std::make_shared<ArrayXXTCoord>(Eigen::Index(1), Eigen::Index(1));
  cart2DX->coeffRef(0) = TCoord(position.x);
  cart2DY->coeffRef(0) = TCoord(position.y);

  motionModel->setEpipole(epipoleList.findEpipole(curPOCOrig, refPOCOrig));
  ArrayXXTCoordPtrPair cart2DShiftOrig = motionModel->modelMotion({cart2DX, cart2DY}, mvOrig_, {position.x, position.y});
  Array2TCoord shiftOrig(std::get<0>(cart2DShiftOrig)->coeff(0), std::get<1>(cart2DShiftOrig)->coeff(0));

  motionModel->setEpipole(epipoleList.findEpipole(curPOCDesired, refPOCDesired));
  ArrayXXTCoordPtrPair cart2DShiftDesired = motionModel->modelMotion({cart2DX, cart2DY}, mvDesired_, {0, 0});
  Array2TCoord shiftDesired(std::get<0>(cart2DShiftDesired)->coeff(0), std::get<1>(cart2DShiftDesired)->coeff(0));

  std::cout << shiftOrig << " -> " << shiftDesired << std::endl;
}

int main(int argc, char* argv[]) {
  testGEDEpipoleMVDerivation();
  return 0;
}
