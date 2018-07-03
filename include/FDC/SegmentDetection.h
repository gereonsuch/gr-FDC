/* -*- c++ -*- */
/* 
 * Copyright 2018 Gereon Such.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifndef INCLUDED_FDC_SEGMENTDETECTION_H
#define INCLUDED_FDC_SEGMENTDETECTION_H

#include <FDC/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace FDC {

    /*!
     * \brief <+description of block+>
     * \ingroup FDC
     *
     */
    class FDC_API SegmentDetection : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<SegmentDetection> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of FDC::SegmentDetection.
       *
       * To avoid accidental use of raw pointers, FDC::SegmentDetection's
       * constructor is in a private implementation
       * class. FDC::SegmentDetection::make is the public interface for
       * creating new instances.
       */
      static sptr make(int ID, int blocklen, int relinvovl, float seg_start, float seg_stop, float thresh, float minchandist, float window_flank_puffer, int maxblocks_to_emit, int channel_deactivation_delay, bool messageoutput, bool fileoutput, std::string path, bool threads, int verbose);
    };

  } // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_SEGMENTDETECTION_H */

