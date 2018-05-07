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

#ifndef INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_IMPL_H
#define INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_IMPL_H

#include <FDC/activity_controlled_channelizer_vcm.h>
#include <gnuradio/fft/fft.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <deque>
#include <complex>
#include <volk/volk.h>
#include <string>
#include <exception>
#include <thread>
#include <limits>
#include <pmt/pmt.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>

#include <iostream>

namespace gr {
namespace FDC {

enum WINDOWTYPES{
    RECTANGULAR,
    HANN,
    RAMP
};

enum VERBOSE{
    NOLOG=0,
    LOGTOCONSOLE=1,
    LOGTOFILE=2
};

const double bwpuffer=1.2;

class channel{
public:
    std::deque< std::vector< gr_complex > > blocks;
    std::vector< std::vector< gr_complex > > windows;
    int ID; //ID of Channel
    int start;
    int size;
    int red_size;
    int ovlskip;
    int relinvovl;
    int count;
    int channelcount; //number of activations
    int part;
    bool active;
    int active_since;
    float curpwr;
    std::string msgID;


    channel(int id, int begin, int width, int red_width, int relovl);
    bool set_power(const gr_complex *v, double thresh);

    void activate();
    void deactivate();

    void push_data(std::vector< gr_complex > &v);
    void push_data(std::vector< std::vector< gr_complex > > &v);
    void get_data(std::vector<gr_complex> &d);

    std::string get_msg_ID();
};

class activity_controlled_channelizer_vcm_impl : public activity_controlled_channelizer_vcm
{
private:
    int veclen;
    std::vector< channel > chans;
    std::deque< std::thread > threads;
    std::vector< gr_complex > hist;
    float threshold;
    int relovl;
    int maximumblocks;
    bool msg;
    bool fileout;
    std::string outputpath;
    pmt::pmt_t msgport;
    bool threading;
    int channelcounter;
    VERBOSE verbose;
    std::string logfile;
    unsigned int blockcount;

    void log(std::string &s);

public:
    activity_controlled_channelizer_vcm_impl(int blocklen, std::vector< std::vector< float > > channels, float thresh, int relinvovl, int maxblocks, bool message, bool fileoutput, std::string path, bool threaded, int v_verbose);
    ~activity_controlled_channelizer_vcm_impl();

    void joinall();
    void (activity_controlled_channelizer_vcm_impl::*check_channels)(const gr_complex *, const gr_complex *);
    void check_channels_singlethread(const gr_complex *lastblock, const gr_complex *curblock);
    void check_channels_multithread(const gr_complex *lastblock, const gr_complex *curblock);
    void tx_data(channel &c);

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

int nextpow2(int k);

void get_channel_data(const gr_complex *v, channel &chan);
void get_channel_data_hist(const gr_complex *hist, const gr_complex *v, channel &chan);

void cr_window(int blocksize, float passbw, float stopbw, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize);
void fftshift(gr_complex *in, gr_complex *out, int sz);

std::string get_time();


} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_ACTIVITY_CONTROLLED_CHANNELIZER_VCM_IMPL_H */

