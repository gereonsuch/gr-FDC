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

#ifndef INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_IMPL_H
#define INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_IMPL_H

#include <gnuradio/FDC/activity_detection_channelizer_vcm.h>
#include <gnuradio/fft/fft.h>
#include <vector>
#include <deque>
#include <array>
#include <complex>
#include <string>
#include <thread>
#include <math.h>
#include <volk/volk.h>
#include <pmt/pmt.h>
#include <exception>
#include <limits>
#include <ctime>
#include <utility>
#include <algorithm>

#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>

namespace gr {
namespace FDC {

const int channel_reserve_puffer = 256;
const float log2_10 = 3.32192809489f;
//const double window_flank_puffer = 0.1;
//const int channel_deactivation_delay = 1;


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

using fipair=std::pair<float, int>;
bool fipair_sort(fipair &a, fipair &b){return a.first>b.first;}

int get_next_int(int a, std::deque<int> &lst);

template<typename T> int nextpow2(T v);
template<typename T> int lastpow2(T v);
template<typename T> bool ispow2(T v);
std::string get_ID_for_msg(int chanID, int segID);
std::string get_current_time();

class segment{
public:
    int ID;
    int active_channels_counter;
    int start;
    int stop;
    int width;
    int blocklen;
    int relinvovl;
    int chan_detection_decimation_factor;
    double window_flank_puffer;

    std::vector< float > power;
    std::deque< struct active_channel > active_channels;

    //outer=widths, mid=phases, inner=samples

    segment(int v_ID, int v_start, int v_stop, int v_blocklen, int v_chan_detection_decimation_factor, int v_relinvovl, double v_window_flank_puffer);

    void detect_channels(const gr_complex *in, float thresh);
    void measure_power(const gr_complex *in);
    void get_active_channels(std::deque< std::array<int,2> > &poss_chans, float thresh);
    void match_active_channels(std::deque< std::array<int,2> > &poss_chans);

    bool activate(int detect_start, int detect_end);
    //void deactivate(int id);

};

enum VERBOSE{
    NOLOG=0,
    LOGTOCONSOLE=1,
    LOGTOFILE=2
};

class activity_detection_channelizer_vcm_impl : public activity_detection_channelizer_vcm
{
private:
    int blocklen;
    std::vector< segment > segments;
    std::vector< float > power;
    std::vector< gr_complex > hist;
    std::vector < std::vector< std::vector<gr_complex> > > windows;
    float thresh;
    double window_flank_puffer;
    int relinvovl;
    int maxblocks;
    int channel_deactivation_delay;
    bool msg;
    pmt::pmt_t outport;
    bool fileoutput;
    std::string path;
    int chan_detection_decimation_factor;
    bool threading;
    VERBOSE verbose;
    std::string logfile;
    unsigned int blockcount;

    void (activity_detection_channelizer_vcm_impl::*detect_channels_in_segments)(const gr_complex *);
    void (activity_detection_channelizer_vcm_impl::*extract_channels_in_segments)(const gr_complex *, const gr_complex *);

    void cr_windows();
    void save_hist(const gr_complex *in);

    void detect_channels_in_segments_singlethread(const gr_complex *);
    void detect_channels_in_segments_threaded(const gr_complex *);

    void extract_channels_in_segments_singlethread(const gr_complex *, const gr_complex *);
    void extract_channels_in_segments_threaded(const gr_complex *, const gr_complex *);

    void process_channel(const gr_complex *, struct active_channel &);
    void process_channel_hist(const gr_complex *, const gr_complex *, struct active_channel &);
    void emit_channel(struct active_channel &, segment &);
    void emit_unfinished_channel(struct active_channel &, segment &);

    void clear_inactive_channels();

    void set_chan_detection_decimation_factor(float minchandist);
    void create_segment(std::vector<float> &v);

    void fftshift(gr_complex *in, gr_complex *out, int sz);

    void log(std::string &s);


public:
    activity_detection_channelizer_vcm_impl(int v_blocklen, std::vector< std::vector< float > > v_segments, float v_thresh, int v_relinvovl, int v_maxblocks, bool v_message, bool v_fileoutput, std::string v_path, bool v_threads, float v_minchandist, int v_channel_deactivation_delay, double v_window_flank_puffer, int v_verbose);
    ~activity_detection_channelizer_vcm_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

} // namespace FDC
} // namespace gr

#endif /* INCLUDED_FDC_ACTIVITY_DETECTION_CHANNELIZER_VCM_IMPL_H */

