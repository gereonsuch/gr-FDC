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

#ifndef INCLUDED_FDC_VECTOR_CUT_VXX_IMPL_H
#define INCLUDED_FDC_VECTOR_CUT_VXX_IMPL_H

#include <FDC/vector_cut_vxx.h>

namespace gr {
namespace FDC {

class vector_cut_vxx_impl : public vector_cut_vxx
{
private:
    int itemsz, inplen, outplen, shift;

public:
    vector_cut_vxx_impl(int itemsize, int veclen, int offset, int blocklen);
    ~vector_cut_vxx_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_VECTOR_CUT_VXX_IMPL_H */

