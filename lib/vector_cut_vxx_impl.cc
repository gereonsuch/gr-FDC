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
#include "vector_cut_vxx_impl.h"

namespace gr {
namespace FDC {

vector_cut_vxx::sptr
vector_cut_vxx::make(int itemsize, int veclen, int offset, int blocklen)
{
    return gnuradio::get_initial_sptr
            (new vector_cut_vxx_impl(itemsize, veclen, offset, blocklen));
}

/*
     * The private constructor
     */
vector_cut_vxx_impl::vector_cut_vxx_impl(int itemsize, int veclen, int offset, int blocklen)
    : gr::sync_block("vector_cut_vxx",
                     gr::io_signature::make(1, 1, itemsize * veclen),
                     gr::io_signature::make(1, 1, itemsize * blocklen))
{
    itemsz=itemsize;
    inplen=itemsz * veclen;
    outplen=itemsz * blocklen;
    shift=offset * itemsz;
}

/*
     * Our virtual destructor.
     */
vector_cut_vxx_impl::~vector_cut_vxx_impl()
{
}

int
vector_cut_vxx_impl::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items)
{
    const char *in = (const char *) input_items[0];
    char *out = (char *) output_items[0];

    for(int i=0;i<noutput_items;i++)
        memcpy(out + i*outplen, in + i*inplen + shift, outplen);


    return noutput_items;
}

} /* namespace FDC */
} /* namespace gr */

