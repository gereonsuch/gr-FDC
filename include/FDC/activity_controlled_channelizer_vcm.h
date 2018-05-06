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


#ifndef INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_H
#define INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_H

#include <FDC/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
  namespace FDC {

    /*!
     * \brief <+description of block+>
     * \ingroup FDC
     *
     */
    class FDC_API activity_controlled_channelizer_vcm : virtual public gr::sync_block
    {
     public:
      typedef boost::shared_ptr<activity_controlled_channelizer_vcm> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of FDC::activity_controlled_channelizer_vcm.
       *
       * To avoid accidental use of raw pointers, FDC::activity_controlled_channelizer_vcm's
       * constructor is in a private implementation
       * class. FDC::activity_controlled_channelizer_vcm::make is the public interface for
       * creating new instances.
       */
      static sptr make(int blocklen, std::vector< std::vector< float > > channels, float thresh, int relinvovl, int maxblocks, bool message, bool fileoutput, std::string path, bool threaded, int verbose);
    };

  } // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_H */

