/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     EncLib.h
    \brief    encoder class (header)
*/

#ifndef __ENCTOP__
#define __ENCTOP__

// Include files
#include "CommonLib/TrQuant.h"
#include "CommonLib/DeblockingFilter.h"
#include "CommonLib/NAL.h"

#include "Utilities/VideoIOYuv.h"

#include "EncCfg.h"
#include "EncGOP.h"
#include "EncSlice.h"
#include "EncHRD.h"
#include "VLCWriter.h"
#include "CABACWriter.h"
#include "InterSearch.h"
#include "IntraSearch.h"
#include "EncSampleAdaptiveOffset.h"
#include "EncReshape.h"
#include "EncAdaptiveLoopFilter.h"
#include "RateCtrl.h"

class EncLibCommon;

//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder class
class EncLib : public EncCfg
{
private:
  // picture
  int                       m_pocLast;                            ///< time index (POC)
  int                       m_receivedPicCount;                   ///< number of received pictures
  uint32_t                  m_codedPicCount;                      ///< number of coded pictures
  PicList&                  m_cListPic;                           ///< dynamic list of pictures
  int                       m_layerId;

  // Multi-model inter prediction
  Projection*               m_projection;                        ///< Fisheye (or 360°) projection
  MVReprojection            m_mvReprojection;                    ///< Motion vector reprojection handler

  // encoder search
  InterSearch               m_cInterSearch;                       ///< encoder search class
  IntraSearch               m_cIntraSearch;                       ///< encoder search class
  // coding tool
  TrQuant                   m_cTrQuant;                           ///< transform & quantization class
  DeblockingFilter          m_deblockingFilter;                   ///< deblocking filter class
  EncSampleAdaptiveOffset   m_cEncSAO;                            ///< sample adaptive offset class
  EncAdaptiveLoopFilter     m_cEncALF;
  HLSWriter                 m_HLSWriter;                          ///< CAVLC encoder
  CABACEncoder              m_CABACEncoder;

  EncReshape                m_cReshaper;                        ///< reshaper class

  // processing unit
  EncGOP                    m_cGOPEncoder;                        ///< GOP encoder
  EncSlice                  m_cSliceEncoder;                      ///< slice encoder
  EncCu                     m_cCuEncoder;                         ///< CU encoder
  // SPS
  ParameterSetMap<SPS>     &m_spsMap;                             ///< SPS. This is the base value
  ParameterSetMap<PPS>     &m_ppsMap;                             ///< PPS. This is the base value
  ParameterSetMap<APS>     &m_apsMap;                             ///< APS. This is the base value
  PicHeader                 m_picHeader;                          ///< picture header
  // RD cost computation
  RdCost                    m_cRdCost;                            ///< RD cost computation class
  CtxCache                  m_CtxCache;                           ///< buffer for temporarily stored context models
  // quality control
  RateCtrl                  m_cRateCtrl;                          ///< Rate control class

  AUWriterIf*               m_AUWriterIf;

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
  CacheModel                m_cacheModel;
#endif

  APS*                      m_apss[ALF_CTB_MAX_NUM_APS];

  APS*                      m_lmcsAPS;
  APS*                      m_scalinglistAPS;

  EncHRD                    m_encHRD;

  bool                      m_doPlt;
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> m_metricTime;
#endif
  int                       m_picIdInGOP;

  VPS*                      m_vps;

  int*                      m_layerDecPicBuffering;

public:
  SPS*                      getSPS( int spsId ) { return m_spsMap.getPS( spsId ); };
  APS**                     getApss() { return m_apss; }
  Ctx                       m_entropyCodingSyncContextState;      ///< leave in addition to vector for compatibility
  PLTBuf                    m_palettePredictorSyncState;

protected:
  void  xGetNewPicBuffer  ( std::list<PelUnitBuf*>& rcListPicYuvRecOut, Picture*& rpcPic, int ppsId ); ///< get picture buffer which will be processed. If ppsId<0, then the ppsMap will be queried for the first match.
  void  xInitOPI(OPI& opi); ///< initialize Operating point Information (OPI) from encoder options
  void  xInitDCI(DCI& dci, const SPS& sps); ///< initialize Decoding Capability Information (DCI) from encoder options
  void  xInitVPS( const SPS& sps ); ///< initialize VPS from encoder options
  void  xInitSPS( SPS& sps );       ///< initialize SPS from encoder options
  void  xInitPPS          (PPS &pps, const SPS &sps); ///< initialize PPS from encoder options
  void  xInitPicHeader    (PicHeader &picHeader, const SPS &sps, const PPS &pps); ///< initialize Picture Header from encoder options
  void  xInitAPS          (APS &aps);                 ///< initialize APS from encoder options
  void  xInitScalingLists ( SPS &sps, APS &aps );     ///< initialize scaling lists
  void  xInitPPSforLT(PPS& pps);
  void  xInitHrdParameters(SPS &sps);                 ///< initialize HRDParameters parameters

  void xInitRPL(SPS &sps);   ///< initialize SPS from encoder options

public:
  EncLib( EncLibCommon* encLibCommon );
  virtual ~EncLib();

  void      create          ( const int layerId );
  void      destroy         ();
  void      init(AUWriterIf *auWriterIf);
  void      deletePicBuffer ();

  // -------------------------------------------------------------------------------------------------------------------
  // member access functions
  // -------------------------------------------------------------------------------------------------------------------

  AUWriterIf*             getAUWriterIf         ()              { return   m_AUWriterIf;           }
  PicList*                getListPic            ()              { return  &m_cListPic;             }
  InterSearch*            getInterSearch        ()              { return  &m_cInterSearch;         }
  IntraSearch*            getIntraSearch        ()              { return  &m_cIntraSearch;         }

  TrQuant*                getTrQuant            ()              { return  &m_cTrQuant;             }
  DeblockingFilter*       getDeblockingFilter   ()              { return  &m_deblockingFilter;     }
  EncSampleAdaptiveOffset* getSAO               ()              { return  &m_cEncSAO;              }
  EncAdaptiveLoopFilter*  getALF                ()              { return  &m_cEncALF;              }
  EncGOP*                 getGOPEncoder         ()              { return  &m_cGOPEncoder;          }
  EncSlice*               getSliceEncoder       ()              { return  &m_cSliceEncoder;        }
  EncHRD*                 getHRD                ()              { return  &m_encHRD;               }
  EncCu*                  getCuEncoder          ()              { return  &m_cCuEncoder;           }
  HLSWriter*              getHLSWriter          ()              { return  &m_HLSWriter;            }
  CABACEncoder*           getCABACEncoder       ()              { return  &m_CABACEncoder;         }

  RdCost*                 getRdCost             ()              { return  &m_cRdCost;              }
  CtxCache*               getCtxCache           ()              { return  &m_CtxCache;             }
  RateCtrl*               getRateCtrl           ()              { return  &m_cRateCtrl;            }


  void                    getActiveRefPicListNumForPOC(const SPS *sps, int POCCurr, int GOPid, uint32_t *activeL0, uint32_t *activeL1);
  void                    selectReferencePictureList(Slice* slice, int POCCurr, int GOPid, int ltPoc);

  void                   setParamSetChanged(int spsId, int ppsId);
  bool                   APSNeedsWriting(int apsId);
  bool                   PPSNeedsWriting(int ppsId);
  bool                   SPSNeedsWriting(int spsId);
  const PPS* getPPS( int Id ) { return m_ppsMap.getPS( Id); }
  const APS*             getAPS(int Id) { return m_apsMap.getPS(Id); }

  EncReshape*            getReshaper()                          { return  &m_cReshaper; }

  ParameterSetMap<APS>*  getApsMap() { return &m_apsMap; }

  bool                   getPltEnc()                      const { return   m_doPlt; }
  void                   checkPltStats( Picture* pic );
#if JVET_O0756_CALCULATE_HDRMETRICS
  std::chrono::duration<long long, ratio<1, 1000000000>> getMetricTime()    const { return m_metricTime; };
#endif
  // -------------------------------------------------------------------------------------------------------------------
  // encoder function
  // -------------------------------------------------------------------------------------------------------------------

  // encode several number of pictures until end-of-sequence
  // snrCSC used for SNR calculations. Picture in original colour space.
  bool encodePrep(bool flush, PelStorage *pcPicYuvOrg, PelStorage *pcPicYuvTrueOrg, PelStorage *pcPicYuvFilteredOrg,
                  PelStorage *pcPicYuvFilteredOrgForFG, const InputColourSpaceConversion snrCSC,
                  std::list<PelUnitBuf *> &rcListPicYuvRecOut, int &numEncoded);

  bool encode(const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut, int &numEncoded);

  bool encodePrep(bool flush, PelStorage *pcPicYuvOrg, PelStorage *pcPicYuvTrueOrg, PelStorage *pcPicYuvFilteredOrg,
                  const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut, int &numEncoded,
                  bool isTff);

  bool encode(const InputColourSpaceConversion snrCSC, std::list<PelUnitBuf *> &rcListPicYuvRecOut, int &numEncoded,
              bool isTff);

  void printSummary(bool isField)
  {
    m_cGOPEncoder.printOutSummary(m_codedPicCount, isField, m_printMSEBasedSequencePSNR, m_printSequenceMSE,
                                  m_printMSSSIM, m_printHexPsnr, m_resChangeInClvsEnabled,
                                  m_spsMap.getFirstPS()->getBitDepths(), m_layerId);
  }

  int getLayerId() const { return m_layerId; }
  VPS* getVPS()          { return m_vps;     }
};

//! \}

#endif // __ENCTOP__

