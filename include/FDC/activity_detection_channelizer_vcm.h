/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
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


#ifndef INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_H
#define INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_H

#include <FDC/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace FDC {

    /*!
     * \brief <+description of block+>
     * \ingroup FDC
     *
     */
    class FDC_API activity_detection_channelizer_vcm : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<activity_detection_channelizer_vcm> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of FDC::activity_detection_channelizer_vcm.
       *
       * To avoid accidental use of raw pointers, FDC::activity_detection_channelizer_vcm's
       * constructor is in a private implementation
       * class. FDC::activity_detection_channelizer_vcm::make is the public interface for
       * creating new instances.
       */
      static sptr make(int v_blocklen, std::vector< std::vector< float > > v_segments, float v_thresh, int v_relinvovl, int v_maxblocks, bool v_message, bool v_fileoutput, std::string v_path, bool v_threads, float v_minchandist, int v_channel_deactivation_delay, double v_window_flank_puffer);
    };

  } // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_H */

