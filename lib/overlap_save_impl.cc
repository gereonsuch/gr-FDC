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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "overlap_save_impl.h"

namespace gr {
  namespace FDC {

    overlap_save::sptr
    overlap_save::make(int itemsize, int outputlen, int overlaplen)
    {
      return gnuradio::get_initial_sptr
        (new overlap_save_impl(itemsize, outputlen, overlaplen));
    }

    /*
     * The private constructor
     */
    overlap_save_impl::overlap_save_impl(int itemsize, int outputlen, int overlaplen)
        : gr::sync_block("overlap_save",
                         gr::io_signature::make(1, 1, itemsize * (outputlen-overlaplen)),
                         gr::io_signature::make(1, 1, itemsize *  outputlen ))
    {
        outplen=outputlen;
        ovllen=overlaplen;
        inplen=outplen-ovllen;

        itemsz=itemsize;

        hist.resize(itemsz*ovllen);
    }

    /*
     * Our virtual destructor.
     */
    overlap_save_impl::~overlap_save_impl()
    {
    }

    int
    overlap_save_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
        const char *in = (const char *) input_items[0];
        char *out = (char *) output_items[0];

        memcpy(out, hist.data(), itemsz*ovllen);
        memcpy(out+itemsz*ovllen, in, itemsz*inplen);

        for(int i=1;i<noutput_items;i++){
            memcpy(out+itemsz*i*outplen, in+itemsz*(i*inplen-ovllen),ovllen*itemsz);
            memcpy(out+itemsz*i*outplen+itemsz*ovllen, in+itemsz*i*inplen, itemsz*inplen);
        }

        memcpy(hist.data(),in+itemsz*(noutput_items*inplen-ovllen),itemsz*ovllen);

        return noutput_items;
    }

  } /* namespace FDC */
} /* namespace gr */

