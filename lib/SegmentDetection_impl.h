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

#ifndef INCLUDED_FDC_SEGMENTDETECTION_IMPL_H
#define INCLUDED_FDC_SEGMENTDETECTION_IMPL_H

#include <gnuradio/FDC/SegmentDetection.h>
#include <gnuradio/fft/fft.h>
#include <volk/volk.h>
#include <pmt/pmt.h>

#include <vector>
#include <deque>
#include <array>
#include <complex>
#include <thread>
#include <math.h>
#include <exception>
#include <limits>
#include <ctime>
#include <utility>
#include <algorithm>

#include <string>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>

#include <boost/lexical_cast.hpp>

namespace gr {
namespace FDC {

using str=std::string;
//using num2str=boost::lexical_cast<std::string>;
using fipair=std::pair<float, size_t>;
bool fipair_sort(fipair &a, fipair &b){return a.first>b.first;}

const float log2_10 = 3.32192809489f;

enum VERBOSE{
    NOLOG=0,
    LOGTOCONSOLE=1,
    LOGTOFILE=2
};

struct active_channel{
    int ID;
    int detect_start;
    int detect_stop;
    int extract_start;
    int extract_stop;
    int extract_width;
    int extract_window;
    int ovlskip;
    int outputsamples;
    int count;
    int phase;
    int phaseincrement;
    int inactive; //int instead of bool for possible turn off for slower deactivation in future
    int part; //only used if unfinished msg or file is emitted
    std::string msg_ID;

    std::deque<std::vector<gr_complex> > data;
};

class SegmentDetection_impl : public SegmentDetection
{
private:
    //base parameters
    int d_ID;
    size_t d_active_channels_counter;
    size_t d_start;
    size_t d_stop;
    size_t d_width;
    size_t d_blocklen;
    size_t d_relinvovl;
    size_t d_blockcount;

    //processing parameters
    bool d_threading;

    //detection parameters
    size_t d_chan_detection_decimation_factor;
    float d_window_flank_puffer;
    float d_thresh;

    //deactivation parameters
    int d_maxblocks;
    int d_channel_deactivation_delay;

    //output parameters
    bool d_msg_output;
    pmt::pmt_t d_msgport;
    bool d_file_output;
    std::string d_fileoutput_path;

    //log
    VERBOSE d_verbose;
    std::string d_logfile;

    std::vector < std::vector< std::vector<gr_complex> > > d_windows;
    std::vector< float > d_power;
    std::deque< struct active_channel > d_active_channels;
    std::vector< gr_complex > d_hist;



    //init and additional methods
    void cr_windows();
    void set_threading(bool threading);
    void save_hist(const gr_complex *in);
    void set_chan_start_stop_width_dec(float start, float stop, float minchandist);
    void fftshift(gr_complex *in, gr_complex *out, int sz);
    void log(std::string s);
    str get_ID_for_msg(size_t chanID);
    str get_current_time();




    //detection methods
    void detect_channels(const gr_complex *in);
    void measure_power(const gr_complex *in);
    void get_active_channels(std::deque< std::array<size_t,2> > &poss_chans);
    void match_active_channels(std::deque< std::array<size_t,2> > &poss_chans);


    bool activate(size_t detect_start, size_t detect_end);


    //signal processing methodss
    void process_active_channels_single_thread(const gr_complex *hist, const gr_complex *sig);
    void process_active_channels_multi_thread(const gr_complex *hist, const gr_complex *sig);

    void process_channel(const gr_complex *, struct active_channel &);
    void process_channel_hist(const gr_complex *, const gr_complex *, struct active_channel &);
    void emit_channel(struct active_channel &);
    void emit_unfinished_channel(struct active_channel &);

    void clear_inactive_channels();

    void (SegmentDetection_impl::*process_active_channels)(const gr_complex *hist, const gr_complex *sig);


public:
    SegmentDetection_impl(int ID, int blocklen, int relinvovl, float seg_start, float seg_stop, float thresh, float minchandist, float window_flank_puffer, int maxblocks_to_emit, int channel_deactivation_delay, bool messageoutput, bool fileoutput, std::string path, bool threads, int verbose);
    ~SegmentDetection_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

template <typename T> float mod_f(T x, T y);
template<typename T> int nextpow2(T v);
template<typename T> int lastpow2(T v);
template<typename T> bool ispow2(T v);

template<typename T> str num2str(T v);

} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_SEGMENTDETECTION_IMPL_H */

