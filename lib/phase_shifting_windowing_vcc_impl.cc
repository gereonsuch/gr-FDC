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
#include "phase_shifting_windowing_vcc_impl.h"

namespace gr {
namespace FDC {

phase_shifting_windowing_vcc::sptr
phase_shifting_windowing_vcc::make(int blocklen, int numphasestates, int shifts, float passbw, float stopbw, int windowtype)
{
    return gnuradio::get_initial_sptr
            (new phase_shifting_windowing_vcc_impl(blocklen, numphasestates, shifts, passbw, stopbw, windowtype));
}

/*
     * The private constructor
     */
phase_shifting_windowing_vcc_impl::phase_shifting_windowing_vcc_impl(int blocklen, int numphasestates, int shifts, float passbw, float stopbw, int windowtype)
    : gr::sync_block("phase_shifting_windowing_vcc",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)*blocklen),
                     gr::io_signature::make(1, 1, sizeof(gr_complex)*blocklen))
{
    if(passbw<=0.0)
        throw std::invalid_argument("PassBw in phase_compensating_windowing_vcc_block must not be < 0");

    if(stopbw<=0.0)
        throw std::invalid_argument("StopBw in phase_compensating_windowing_vcc_block must not be < 0");

    if(stopbw<passbw)
        throw std::invalid_argument("StopBw must not be < PassBw in phase_compensating_windowing_vcc block");

    blocksize=blocklen;
    relinvovl=numphasestates;
    counter=0;
    shift=((shifts % relinvovl) + relinvovl) % relinvovl; //prevent negative shift


    //phase corr factors to each window
    cr_win(windowtype, blocksize, passbw, stopbw, windows, relinvovl, 1, false);
}

/*
     * Our virtual destructor.
     */
phase_shifting_windowing_vcc_impl::~phase_shifting_windowing_vcc_impl()
{
}

int
phase_shifting_windowing_vcc_impl::work(int noutput_items,
                                        gr_vector_const_void_star &input_items,
                                        gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];
    gr_complex *out = (gr_complex *) output_items[0];

    for(int i=0;i<noutput_items;i++){
        volk_32fc_x2_multiply_32fc(out+i*blocksize, in+i*blocksize, windows[counter].data() , blocksize); //multiply window
        counter=(counter+shift)%relinvovl; //increment counter to get next phased window on next iteration
    }

    return noutput_items;
}

} /* namespace FDC */
} /* namespace gr */

