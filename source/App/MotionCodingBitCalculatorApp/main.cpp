//
// Created by regensky on 02.03.21.
//

#include <iostream>
#include <bitset>
#include <regex>
#include "Coordinate.h"
#include "Projection.h"
#include "MVReprojection.h"
#include "Utilities/program_options_lite.h"
#include "VLCReader.h"
#include "VLCWriter.h"
#include "BitStream.h"
namespace po = df::program_options_lite;
using namespace std;

EpipoleList                 g_epipoleList;
EpipoleList::PredictionMode g_epipolePredictionMode;
int g_expGolombK;

static const struct MapStrToEpipolePredictionMode
{
  const char                 *str;
  EpipoleList::PredictionMode value;
} strToEpipolePredictionMode[] = { { "none", EpipoleList::PredictionMode::NONE },
                                   { "closest", EpipoleList::PredictionMode::CLOSEST },
                                   { "closest_old", EpipoleList::PredictionMode::CLOSEST_OLD } };

template<typename T, typename P> static istream &readStrToEnum(P map[], uint32_t mapLen, istream &in, T &val)
{
  string str;
  in >> str;

  for (uint32_t i = 0; i < mapLen; i++)
  {
    if (str == map[i].str)
    {
      val = map[i].value;
      goto found;
    }
  }
  /* not found */
  in.setstate(ios::failbit);
found:
  return in;
}

static inline istream &operator>>(istream &in, EpipoleList::PredictionMode &predictionMode)
{
  return readStrToEnum(strToEpipolePredictionMode,
                       sizeof(strToEpipolePredictionMode) / sizeof(*strToEpipolePredictionMode), in, predictionMode);
}

// void testGolombEncoding() {
//   for (int i = 0; i < 3e7; ++i)
//   {
//     OutputBitstream outBitstream;
//     HLSWriter writer;
//     writer.setBitstream(&outBitstream);
//
//     //  int i = 1;
//     writer.xWriteExpColomb(i, 19);
//     outBitstream.writeAlignZero();
//
//     InputBitstream inBitstream;
//     for (auto &byte: outBitstream.getFIFO())
//     {
//       std::cout << "Bitstream: " << bitset<8>(byte) << std::endl;
//       inBitstream.getFifo().push_back(byte);
//     }
//
//     HLSyntaxReader reader;
//     reader.setBitstream(&inBitstream);
//
//     uint32_t val;
//     reader.xReadExpGolomb(val, 19);
//     std::cout << i << "->" << val << std::endl;
//
//     if (val != i)
//     {
//       std::cout << "BAD!" << std::endl;
//       return;
//     }
//   }
// }

int getExpGolombBits(std::uint32_t uiCode, int k)
{
  uint32_t y      = (uiCode >> k) + 1;
  uint32_t length = floorLog2(y) + 1;
  int      bits   = int(2 * length - 1 + k);
  return bits;
}

int getUvlcBits(std::uint32_t uiCode)
{
  uint32_t length = 1;
  uint32_t temp   = ++uiCode;

  CHECK(!temp, "Integer overflow");

  while (1 != temp)
  {
    temp >>= 1;
    length += 2;
  }

  int bits = int((length >> 1) + ((length + 1) >> 1));
  return bits;
}

int getEpipoleBits(EpipoleList &epipoleList, const std::vector<int> &pocCodingOrder)
{
  int bits = 0;
  for (auto poc: pocCodingOrder)
  {
    const auto epipolePredictor = epipoleList.derivePredictor(poc);
    epipoleList.makeAvailable(poc);
    const auto epipole = epipoleList.findEpipole(poc, -1);
    const auto epipoleSpherical = CoordinateConversion::cartesianToSpherical(epipole);
    const auto epipoleSphericalTrim = Array2TCoord(epipoleSpherical.coeff(1), epipoleSpherical.coeff(2));
    const Array2Fixed epipoleDelta = FloatingFixedConversion::floatingToFixed(epipoleSphericalTrim, EPIPOLE_PRECISION_FIXED) - FloatingFixedConversion::floatingToFixed(epipolePredictor, EPIPOLE_PRECISION_FIXED);

    switch (epipoleList.getPredictionMode())
    {
    case EpipoleList::PredictionMode::NONE:
    {
      for (int i = 0; i < 2; ++i)
      {
        bits += EPIPOLE_PRECISION_FIXED + 2;
      }
      break;
    }
    case EpipoleList::PredictionMode::CLOSEST:
    {
      for (int i = 0; i < 2; ++i)
      {
        bits += 1;                                             // ph_epipole_delta_i_sign_flag
        bits += getExpGolombBits(abs(epipoleDelta.coeff(i)), g_expGolombK);   // ph_epipole_delta_i_abs_value
      }
      break;
    }
    case EpipoleList::PredictionMode::CLOSEST_OLD:
    {
      bits += 1;   // ph_signal_epipole_delta_flag
      for (int i = 0; i < 2; ++i)
      {
        bits += 1;                                         // ph_epipole_delta_i_sign_flag
        bits += getUvlcBits(abs(epipoleDelta.coeff(i)));   // ph_epipole_delta_i_abs_value
      }
      break;
    }
    }
  }

  return bits;
}

void parseEpipole(po::Options &options, const string &argv, po::ErrorReporter &errorReporter)
{
  regex regex_epipole_entry(
    R"((?:-?\d+)(\s*,?\s*)(?:-?\d+)(\s*,?\s*)(-?\d+(?:\.\d*)?|\.\d+)(\s*,?\s*)(-?\d+(?:\.\d*)?|\.\d+)(\s*,?\s*)(-?\d+(?:\.\d*)?|\.\d+))");
  for (sregex_iterator i = { argv.begin(), argv.end(), regex_epipole_entry }; i != sregex_iterator(); ++i)
  {
    smatch epipole_entry_match     = *i;
    string epipole_entry_match_str = epipole_entry_match.str();
    epipole_entry_match_str        = regex_replace(epipole_entry_match_str, regex(R"(([\s,]+))"), " ");
    istringstream iss(epipole_entry_match_str);
    int           curPOC, refPOC;
    TCoord        x, y, z;
    iss >> curPOC;
    iss >> refPOC;
    iss >> x;
    iss >> y;
    iss >> z;
    g_epipoleList.addEpipole({ x, y, z }, curPOC, refPOC);
  }
}

int main(int argc, char *argv[])
{
  po::Options opts;
  opts.addOptions()("c", po::parseConfigFile, "configuration file name")(
    "Epipole", [](po::Options &opts, const string &argv, po::ErrorReporter &er) { parseEpipole(opts, argv, er); },
    "Epipole list entry as (curPOC, refPOC, x, y, z).")("EpipolePredictionMode", g_epipolePredictionMode,
                                                        EpipoleList::PredictionMode::NONE,
                                                        "Epipole prediction mode (none, closest, closest_old)")
    ("ExpGolombK", g_expGolombK, 19, "Exp-Golomb parameter k");

  po::setDefaults(opts);
  po::scanArgv(opts, argc, (const char **) argv);

  //  std::cout << "EpipolePredictionMode: " << int(g_epipolePredictionMode) << std::endl;
  //  g_epipoleList.printSummary();

  g_epipoleList.addEpipole({ 0, 0, 0 }, -1, -1, true);
  g_epipoleList.setPredictionMode(g_epipolePredictionMode);
  std::vector<int> pocCodingOrder = { 16, 8,  4,  2,  1,  3,  6,  5,  7,  12, 10, 9,  11, 14, 13, 15,
                                      24, 20, 18, 17, 19, 22, 21, 23, 28, 26, 25, 27, 30, 29, 31 };
  int              bits           = getEpipoleBits(g_epipoleList, pocCodingOrder);
  std::cout << bits << std::endl;
  return 0;
}
