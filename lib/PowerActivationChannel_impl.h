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

#ifndef INCLUDED_FDC_POWERACTIVATIONCHANNEL_IMPL_H
#define INCLUDED_FDC_POWERACTIVATIONCHANNEL_IMPL_H

#include <gnuradio/FDC/PowerActivationChannel.h>

#include <complex>
#include <deque>
#include <vector>
#include <gnuradio/fft/fft.h>
#include <string>
#include <exception>
#include <volk/volk.h>
#include <math.h>
#include <pmt/pmt.h>
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <ctime>


namespace gr {
namespace FDC {

using str=std::string;

enum VERBOSE{
    NOLOG=0,
    LOGTOCONSOLE=1,
    LOGTOFILE=2
};

class PowerActivationChannel_impl : public PowerActivationChannel
{
private:
    //signal parameters
    int blocklen;
    int relinvovl;
    int extract_start;
    int extract_stop;
    int extract_width;
    int output_len;
    int output_ovl_offset;
    int measure_start;
    int measure_stop;
    int maxblocks;
    int deactivation_delay;
    float thresh;

    bool active;
    float lastpower;

    //windows
    std::vector< std::vector<gr_complex> > windows;

    //storage parameters
    std::vector<gr_complex> hist;
    std::deque< std::vector<gr_complex> > blocks;
    int count;
    int phase;
    int deltaphase;

    //output parameters
    int ID;
    str msgID;
    int part;
    int finished_channels;
    bool msg;
    pmt::pmt_t msgport;
    bool fileoutput;
    str path;

    //debugging parameters
    int blockcount;
    VERBOSE verbosemode;
    str logfile;

    //init methods
    void set_startstop(float cfreq, float bw);
    void cr_windows();
    void set_thresh(float v_thresh);
    str create_ID();

    //dsp methods
    bool measure_power(const gr_complex *sig);
    void process_channel(const gr_complex *sig);
    void emit_data(bool fin=true);

    void activate(const gr_complex *sighist, const gr_complex *sig);
    void deactivate();

    void save_hist(const gr_complex *sig);

    //helping methods
    void log(str s);
    bool ispow2(int k);
    int nextpow2(int k);
    str get_current_time();
    void fftshift(const gr_complex *src, gr_complex *dest, int sz);

public:
    PowerActivationChannel_impl(int v_blocklen, float v_cfreq, float v_bw, int v_relinvovl, float v_thresh, int v_maxblocks, int v_deactivation_delay, bool v_msg, bool v_fileoutput, std::string v_path, int verbose, int v_ID);
    ~PowerActivationChannel_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_POWERACTIVATIONCHANNEL_IMPL_H */

