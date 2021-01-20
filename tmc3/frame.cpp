/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2017-2021, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "frame.h"

#include "PCCMisc.h"

namespace pcc {

//============================================================================

void
CloudFrame::setParametersFrom(const SequenceParameterSet& sps)
{
  this->geometry_axis_order = sps.geometry_axis_order;
  this->outputOrigin = sps.seqBoundingBoxOrigin;
  this->outputUnitLength = reciprocal(sps.seq_geom_scale);
  this->outputUnit = sps.seq_geom_scale_unit_flag;
  this->attrDesc = sps.attributeSets;
}

//============================================================================

void
scaleGeometry(
  PCCPointSet3& cloud, const SequenceParameterSet::GlobalScale& globalScale)
{
  // Conversion to rational simplifies the globalScale expression.
  Rational gs = globalScale;

  // Nothing to do if scale factor is 1.
  if (gs.numerator == gs.denominator)
    return;

  // NB: by definition, gs.denominator is a power of two.
  int gsDenominatorLog2 = ilog2(uint32_t(gs.denominator));

  // The scaling here is equivalent to the integer conformance output
  size_t numPoints = cloud.getPointCount();
  for (size_t i = 0; i < numPoints; i++) {
    auto& pos = cloud[i];
    pos = (pos * gs.numerator + (gs.denominator >> 1)) >> gsDenominatorLog2;
  }
}

//============================================================================

}  // namespace pcc