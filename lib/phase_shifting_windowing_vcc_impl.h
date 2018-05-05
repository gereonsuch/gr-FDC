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

#ifndef INCLUDED_FDC_PHASE_SHIFTING_WINDOWING_VCC_IMPL_H
#define INCLUDED_FDC_PHASE_SHIFTING_WINDOWING_VCC_IMPL_H

#include <FDC/phase_shifting_windowing_vcc.h>
#include <volk/volk.h>
#include <exception>
#include <math.h>
#include <complex>
#include <vector>
#include "windows.h"

namespace gr {
namespace FDC {



enum WINDOWTYPES{
    RECTANGULAR,
    HANN,
    RAMP
};


class phase_shifting_windowing_vcc_impl : public phase_shifting_windowing_vcc
{
private:
    int blocksize, relinvovl, counter, shift;

    std::vector< std::vector<gr_complex> > windows;

public:
    phase_shifting_windowing_vcc_impl(int blocklen, int numphasestates, int shifts, float passbw, float stopbw, int windowtype);
    ~phase_shifting_windowing_vcc_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};


} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_PHASE_SHIFTING_WINDOWING_VCC_IMPL_H */

