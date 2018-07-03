/* -*- c++ -*- */

#define FDC_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "FDC_swig_doc.i"

%{
#include "FDC/overlap_save.h"
#include "FDC/phase_shifting_windowing_vcc.h"
#include "FDC/vector_cut_vxx.h"
#include "FDC/activity_detection_channelizer_vcm.h"
#include "FDC/PowerActivationChannel.h"
#include "FDC/SegmentDetection.h"
%}


%include "FDC/overlap_save.h"
GR_SWIG_BLOCK_MAGIC2(FDC, overlap_save);
%include "FDC/phase_shifting_windowing_vcc.h"
GR_SWIG_BLOCK_MAGIC2(FDC, phase_shifting_windowing_vcc);
%include "FDC/vector_cut_vxx.h"
GR_SWIG_BLOCK_MAGIC2(FDC, vector_cut_vxx);

%include "FDC/activity_detection_channelizer_vcm.h"
GR_SWIG_BLOCK_MAGIC2(FDC, activity_detection_channelizer_vcm);
%include "FDC/PowerActivationChannel.h"
GR_SWIG_BLOCK_MAGIC2(FDC, PowerActivationChannel);
%include "FDC/SegmentDetection.h"
GR_SWIG_BLOCK_MAGIC2(FDC, SegmentDetection);
