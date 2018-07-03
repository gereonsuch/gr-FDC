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
#include "SegmentDetection_impl.h"

namespace gr {
namespace FDC {

SegmentDetection::sptr
SegmentDetection::make(int ID, int blocklen, int relinvovl, float seg_start, float seg_stop, float thresh, float minchandist, float window_flank_puffer, int maxblocks_to_emit, int channel_deactivation_delay, bool messageoutput, bool fileoutput, str path, bool threads, int verbose)
{
    return gnuradio::get_initial_sptr
            (new SegmentDetection_impl(ID, blocklen, relinvovl, seg_start, seg_stop, thresh, minchandist, window_flank_puffer, maxblocks_to_emit, channel_deactivation_delay, messageoutput, fileoutput, path, threads, verbose));
}

/*
     * The private constructor
     */
SegmentDetection_impl::SegmentDetection_impl(int ID, int blocklen, int relinvovl, float seg_start, float seg_stop, float thresh, float minchandist, float window_flank_puffer, int maxblocks_to_emit, int channel_deactivation_delay, bool messageoutput, bool fileoutput, str path, bool threads, int verbose)
    : gr::sync_block("SegmentDetection",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)*blocklen),
                     gr::io_signature::make(0, 0, 0))
{
    d_ID=ID;

    //set logging and runtime mode
    if((VERBOSE) verbose==LOGTOFILE){
        d_verbose=LOGTOFILE;
        d_logfile=str("gr-FDC.ActDetChan.ID_") + num2str(d_ID) + str(".log");
        FILE *f=fopen(d_logfile.c_str(), "w");
        if(!f)
            std::cerr << "Logfile not writable: " << d_logfile << std::endl;
        else
            fwrite("\n", sizeof(char), 1, f);
        fclose(f);
    }else if((VERBOSE) verbose==LOGTOCONSOLE)
        d_verbose=LOGTOCONSOLE;
    else
        d_verbose=NOLOG;

    set_threading(threads);

    //init base and detection parameters

    d_blocklen=blocklen;
    if(!ispow2(d_blocklen))
        throw std::invalid_argument("Blocklen must be Power of 2. ");

    d_relinvovl=relinvovl;
    if(!ispow2(d_relinvovl))
        throw std::invalid_argument("Relinvovl must be Power of 2. ");

    if(thresh<0.0f)
        throw std::invalid_argument("Threshold is interpreted as dB and must be greater zero to detect channels accordingly. ");
    d_thresh=(float) pow( 10.0, (double)thresh/10.0 );

    d_hist.resize(d_blocklen, gr_complex(0.0f, 0.0f));

    d_maxblocks=maxblocks_to_emit;
    d_channel_deactivation_delay=channel_deactivation_delay;

    d_window_flank_puffer=window_flank_puffer;
    if(d_window_flank_puffer<0.0)
        throw std::invalid_argument("Window flank puffer must not be smaller 0.0. \n");

    //set detection start, stop, width and detection decimation factor
    set_chan_start_stop_width_dec(seg_start, seg_stop, minchandist);

    //create all possible windows for blocklen
    cr_windows();



    //set output parameters
    d_msg_output=messageoutput;
    if(d_msg_output){
        d_msgport=pmt::intern("msgout");
        message_port_register_out(d_msgport);
    }

    d_file_output=fileoutput;
    if(d_file_output){
        //possible plausibility/writability check here?
        d_fileoutput_path=path;
    }

    log(str("Threshold               ") + num2str(d_thresh));
    log(str("decimation factor       ") + num2str(d_chan_detection_decimation_factor));
    log(str("start                   ") + num2str(d_start));
    log(str("stop                    ") + num2str(d_stop));
    log(str("width                   ") + num2str(d_width));


    //init counters
    d_active_channels_counter=0;
    d_blockcount=0;



}

/*
     * Our virtual destructor.
     */
SegmentDetection_impl::~SegmentDetection_impl()
{
}

int
SegmentDetection_impl::work(int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];

    const gr_complex *sig;
    const gr_complex *sig_hist = d_hist.data(); //pointer to last block, initialized with saved history.

    for(int i=0;i<noutput_items;i++){ //1 item i = 1 block
        sig=in+i*d_blocklen; //sig is current block

        //channel detection
        detect_channels(sig);

        //process of all channels HERE
        (this->*(process_active_channels))(sig_hist, sig);

        //finally iterate history pointer
        sig_hist=sig;

        d_blockcount++;
    }

    //saving last block to history puffer
    save_hist(sig);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

void SegmentDetection_impl::detect_channels(const gr_complex *in){
    measure_power(in);

    std::deque< std::array<size_t,2> > poss_chans;

    //detect all possible channels, wether or not already active
    get_active_channels(poss_chans);

    //match active channels
    match_active_channels(poss_chans);



}

void SegmentDetection_impl::measure_power(const gr_complex *in){
    //calculate the decimated power over the segment

    size_t N=d_power.size();
    unsigned int alignment = volk_get_alignment();
    float* tmp = (float*)volk_malloc(sizeof(float)*d_width, alignment);

    volk_32fc_magnitude_squared_32f(tmp, in+d_start, d_width);

    for(size_t i=0;i<N;i++)
        volk_32f_accumulator_s32f(d_power.data()+i,
                                  tmp+i*d_chan_detection_decimation_factor,
                                  d_chan_detection_decimation_factor);

    volk_free(tmp);
}

void SegmentDetection_impl::get_active_channels(std::deque<std::array<size_t, 2> > &poss_chans){
    std::deque< fipair > riseedge;
    std::deque< size_t > falledge; //no priorisation

    float inversethresh=1.0f / d_thresh;

    size_t N=d_power.size()-1;
    unsigned int alignment = volk_get_alignment();
    float* tmp = (float*)volk_malloc(sizeof(float)*N, alignment);

    //calculate all powerdifferences and get possible rising and falling edges
    volk_32f_x2_divide_32f(tmp, d_power.data()+1, d_power.data(), N);

    for(size_t i=0;i<N;i++){
        if(tmp[i] > d_thresh) riseedge.push_back( {tmp[i], i*d_chan_detection_decimation_factor + d_start} );
        else if(tmp[i] < inversethresh) falledge.push_back( (i+1)*d_chan_detection_decimation_factor + d_start );
    }

    volk_free(tmp);


    //sort rising edges
    std::sort(riseedge.begin(), riseedge.end(), fipair_sort);

    size_t poss_start;
    std::deque<size_t>::iterator next_end;
    bool breaking=false;
    while(riseedge.size()){
        poss_start=riseedge.front().second;
        riseedge.pop_front();

        next_end=std::upper_bound(falledge.begin(), falledge.end(), poss_start);

        if(next_end-falledge.begin()>=falledge.size()) // || *next_end<=poss_start) //this is excluded by std::upper_bound
            continue; //discard if none found

        breaking=false;
        for(std::array<size_t, 2> &arr: poss_chans)
            if( poss_start<arr[1] && *next_end>=arr[0] ){
                breaking=true;
                break; //if overlapping with any, discard
            }

        if(breaking)
            continue;

        //no check failed, add it to poss_chans
        poss_chans.push_back( {poss_start, *next_end} );
    }
}

void SegmentDetection_impl::match_active_channels(std::deque<std::array<size_t, 2> > &poss_chans){
    if(poss_chans.empty()){
        for(struct active_channel &c:d_active_channels)
            c.inactive+=1;
        return; //nothing to do since poss_chans is empty
    }

    int pc_start, pc_end;
    bool inactive;
    int i;

    for(struct active_channel &c: d_active_channels){
        inactive=true;
        i=0;
        while(i<poss_chans.size()){
            pc_start=poss_chans.begin()[i][0];
            pc_end=poss_chans.begin()[i][1];

            if( pc_start<c.detect_stop && pc_end>=c.detect_start ){
                //this possible channel is overlapping an already active channel.
                c.inactive=0; //this channel is marked as active!
                inactive=false;
                poss_chans.erase(poss_chans.begin()+i);
            }else
                i++;
        }
        //if no channel overlapping to currently active channel is found, it's deactivation counter is increased.
        if(inactive)
            c.inactive+=1;
    }

    //all remaining elements in poss_chans are new channels.
    //poss_chans is necessarily without overlapping possible channels.
    if(poss_chans.size())
        for(std::array<size_t,2> &pc: poss_chans)
            if( !activate(pc[0], pc[1]) ){
                //Error occured, give additional infos
                std::cerr << "Possible Chans[ " << poss_chans.size() <<" ]: ";
                for(std::array<size_t, 2> &arr: poss_chans)
                    std::cerr << "[" << arr[0] << ", " << arr[1] << "], ";
                std::cerr << std::endl;
            }
}

bool SegmentDetection_impl::activate(size_t detect_start, size_t detect_end){
    //Creates a new channel in the segment. Start and stop of channel may exceed the segment, since
    //the extraction flank exceeds the detection bandwidth.

    size_t detect_width=detect_end - detect_start;
    size_t extract_mid=detect_start + detect_width/2;
    size_t extract_width=nextpow2( (size_t) ceil((double)detect_width * (1.0+2.0*d_window_flank_puffer)) );

    if(extract_width>d_blocklen){
        str s=str("channel width exceeds blocklen. start=") + num2str(detect_start) +
                str(", end=") + num2str(detect_end) +
                str(", width=")+ num2str(detect_width)+
                str("\nresulting in width=")+num2str(extract_width) +
                str(" blocklen=")+num2str(d_blocklen)+str("\n\n");
        //throw std::invalid_argument(s.c_str());
        //instead of throwing an error, log everything here to cerr and skip this channel.
        std::cerr << s << std::endl;
        return false; //can be caught where it's called
    }

    int extract_start=extract_mid-extract_width/2;
    int extract_end=extract_mid+extract_width/2;

    if(extract_start<0){
        extract_start=0;
        extract_end=extract_width;
    }
    if(extract_end>d_blocklen){
        extract_end=d_blocklen;
        extract_start=d_blocklen-extract_width;
    }

    struct active_channel c;
    c.ID=d_active_channels_counter++; //increment counter and take last num as ID for new channel.
    c.detect_start=detect_start;
    c.detect_stop=detect_end;
    c.extract_start=extract_start;
    c.extract_stop=extract_end;
    c.extract_width=extract_width;
    c.extract_window=(int) log2((double) extract_width);
    c.ovlskip=extract_width / d_relinvovl;
    c.outputsamples=c.extract_width-c.ovlskip;
    c.count=0;
    c.phase=0;
    c.phaseincrement=extract_start%d_relinvovl;
    c.inactive=-1; //-1 is initialized.
    c.part=0;
    c.msg_ID=get_ID_for_msg(c.ID);

    c.data.clear();

    d_active_channels.push_back( c );

    return true;
}

void SegmentDetection_impl::process_active_channels_single_thread(const gr_complex *hist, const gr_complex *sig){
    for(struct active_channel &c: d_active_channels){
        if(c.inactive < 0)
            process_channel_hist(hist, sig, c);
        else if(c.inactive > d_channel_deactivation_delay)
            emit_channel(c);
        else
            process_channel(sig, c);
    }

    //if one channel was active longer than specified maxblocks, emit it partly.
    //To avoid conflicts, this is done after finishing all dsp and regular emitting tasks

    if(d_maxblocks>=0)
        for(struct active_channel &c: d_active_channels)
            if(c.data.size()>=d_maxblocks)
                emit_unfinished_channel(c);

    clear_inactive_channels();
}

void SegmentDetection_impl::process_active_channels_multi_thread(const gr_complex *hist, const gr_complex *sig){
    std::deque<std::thread> threads;

    for(struct active_channel &c: d_active_channels){
        if(c.inactive < 0)
            threads.push_back( std::thread( &SegmentDetection_impl::process_channel_hist, this, hist, sig, std::ref(c) ) );
        else if(c.inactive > d_channel_deactivation_delay)
            threads.push_back( std::thread( &SegmentDetection_impl::emit_channel, this, std::ref(c) ) );
        else
            threads.push_back( std::thread( &SegmentDetection_impl::process_channel, this, sig, std::ref(c) ) );
    }

    for(std::thread &t: threads)
        t.join();
    threads.clear();

    //if one channel was active longer than specified maxblocks, emit it partly.
    //To avoid conflicts, this is done after finishing all dsp and regular emitting tasks

    if(d_maxblocks>=0)
        for(struct active_channel &c: d_active_channels)
            if(c.data.size()>=d_maxblocks)
                threads.push_back( std::thread( &SegmentDetection_impl::emit_unfinished_channel, this, std::ref(c) ) );

    for(std::thread &t: threads)
        t.join();
    threads.clear();

    clear_inactive_channels();

}

void SegmentDetection_impl::process_channel(const gr_complex *sig, active_channel &c){
    unsigned int alignment = volk_get_alignment();
    gr_complex* tmp = (gr_complex*)volk_malloc(sizeof(gr_complex)*c.extract_width, alignment);


    gr::fft::fft_complex ifft(c.extract_width, false, 1); //gnuradio fft object

    //multiply with window
    volk_32fc_x2_multiply_32fc(tmp,
                               sig+c.extract_start,
                               d_windows[c.extract_window][c.phase].data(),
            c.extract_width);

    //fftshift
    fftshift(tmp, ifft.get_inbuf(), c.extract_width);

    //ifft
    ifft.execute();


    //copy ifft output buffer without overlap to data storeage puffer.
    c.data.push_back( {} );
    c.data.back().resize(c.outputsamples);
    memcpy(c.data.back().data(), ifft.get_outbuf()+c.ovlskip, c.outputsamples*sizeof(gr_complex));

    //increment counters.
    c.count++;
    c.phase=(c.phase+c.phaseincrement)%d_relinvovl;

    volk_free(tmp);
}

void SegmentDetection_impl::process_channel_hist(const gr_complex *hist, const gr_complex *sig, active_channel &c){
    process_channel(hist, c);
    process_channel(sig, c);
    c.inactive=0;
}

void SegmentDetection_impl::emit_channel(active_channel &c){
    std::vector< gr_complex > d; //puffer vector.
    d.resize( c.data.size()*c.outputsamples );

    for(int i=0;i<c.data.size();i++) //could also emit all... but maybe following blocks need a fixed length invoming vector if parted.
        memcpy( d.data()+i*c.outputsamples, (c.data.begin()+i)->data(), c.outputsamples*sizeof(gr_complex) );

    c.data.clear();

    if(d_msg_output){ //emit channel data to msgport
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern(c.msg_ID));
        dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(true));
        if(c.part>0)
            dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(c.part)); //add part information if emitted previously
        //add samprate, carrierfrequency, sps, ... if known.
        dict = pmt::dict_add(dict, pmt::intern("rel_bw"), pmt::from_double((double)(c.extract_width)/(double)d_blocklen));
        dict = pmt::dict_add(dict, pmt::intern("rel_cfreq"), pmt::from_double((double)(c.extract_start+c.extract_stop)/2.0/(double)d_blocklen));
        dict = pmt::dict_add(dict, pmt::intern("blockstart"), pmt::from_long(d_blockcount-c.count));
        dict = pmt::dict_add(dict, pmt::intern("blockend"), pmt::from_long(d_blockcount));
        dict = pmt::dict_add(dict, pmt::intern("vectorstart"), pmt::from_long(c.extract_start));
        dict = pmt::dict_add(dict, pmt::intern("vectorend"), pmt::from_long(c.extract_stop));

        message_port_pub(d_msgport, pmt::cons(dict, pmt::init_c32vector( d.size(), d ))); //emit as PDU
    }
    if(d_file_output){ //write channel data to file
        std::string filename=d_fileoutput_path+std::string("/")+c.msg_ID+std::string(".fin");
        FILE* fh=fopen( filename.c_str(), "wb" );
        if(!fh)
            std::cerr << "Cannot write to file " << filename << std::endl; //dont want to throw errors here...
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), fh);
        fclose(fh);
    }

    //channel is erased in other method.

    if(d_verbose){
        std::string s=c.msg_ID+std::string(".fin: ");
        s+=std::string("start=")+num2str(c.extract_start)+
                std::string(", stop=")+num2str(c.extract_stop)+
                std::string(", blockstart=")+num2str(d_blockcount-c.count)+
                std::string(", blockend=")+num2str(d_blockcount);
        log(s);
    }
}

void SegmentDetection_impl::emit_unfinished_channel(active_channel &c){
    if(d_maxblocks<0 || c.data.size()<d_maxblocks) //in this case, this method should not be called. but nevertheless check it.
        return;

    //emit maxblocks or everything if maxblocks==0
    int Ntxblocks=d_maxblocks==0?c.data.size():d_maxblocks;

    if(Ntxblocks<=0)
        return; //just to be sure...

    std::vector< gr_complex > d; //puffer vector.
    d.resize( Ntxblocks*c.outputsamples );

    for(int i=0;i<Ntxblocks;i++)
        memcpy( d.data()+i*c.outputsamples, (c.data.begin()+i)->data(), c.outputsamples*sizeof(gr_complex) );

    c.data.erase(c.data.begin(), c.data.begin()+Ntxblocks); //erase all copied vectors.

    if(d_msg_output){ //emit channel data to msgport
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern(c.msg_ID));
        dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(false));
        dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(c.part));
        //add samprate, carrierfrequency, sps, ... if known.
        dict = pmt::dict_add(dict, pmt::intern("rel_bw"), pmt::from_double((double)(c.extract_width)/(double)d_blocklen));
        dict = pmt::dict_add(dict, pmt::intern("rel_cfreq"), pmt::from_double((double)(c.extract_start+c.extract_stop)/2.0/(double)d_blocklen));
        dict = pmt::dict_add(dict, pmt::intern("blockstart"), pmt::from_long(d_blockcount-c.count));
        dict = pmt::dict_add(dict, pmt::intern("blockend"), pmt::from_long(d_blockcount));
        dict = pmt::dict_add(dict, pmt::intern("vectorstart"), pmt::from_long(c.extract_start));
        dict = pmt::dict_add(dict, pmt::intern("vectorend"), pmt::from_long(c.extract_stop));

        message_port_pub(d_msgport, pmt::cons(dict, pmt::init_c32vector( d.size(), d ))); //emit as PDU

    }
    if(d_file_output){ //write channel data to file
        std::string filename=d_fileoutput_path+std::string("/")+c.msg_ID+std::string(".parted.")+std::to_string(c.part);
        FILE* fh=fopen( filename.c_str(), "wb" );
        if(!fh)
            std::cerr << "Cannot write to file " << filename << std::endl; //dont want to throw errors here...
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), fh);
        fclose(fh);
    }

    c.part++;

    if(d_verbose){
        std::string s=c.msg_ID+std::string(".part: ");
        s+=std::string("start=")+num2str(c.extract_start)+
                std::string(", stop=")+num2str(c.extract_stop)+
                std::string(", part=")+num2str(c.part)+
                std::string(", blockstart=")+num2str(d_blockcount-c.count)+
                std::string(", blockend=")+num2str(d_blockcount);
        log(s);
    }
}

void SegmentDetection_impl::clear_inactive_channels(){
    int i=0;
    while(i<d_active_channels.size()){
        if( (d_active_channels.begin()+i)->inactive > d_channel_deactivation_delay )
            d_active_channels.erase( d_active_channels.begin()+i );
        else
            i++;
    }
}

void SegmentDetection_impl::cr_windows(){
    int num_window_sizes=(int) log2((double) d_blocklen)+1;
    int winwidth;
    int puffersamples;
    float flank;

    d_windows.resize(num_window_sizes);

    //d_windows is vec[vec[vec[complex]]]
    //meaning: window_length, phase_shift, samples[complex]

    for(int winsize=0;winsize<num_window_sizes;winsize++){
        d_windows[winsize].resize(d_relinvovl);

        winwidth=1<<winsize;

        puffersamples=(int)(d_window_flank_puffer*(double)winwidth);

        for(int i=0;i<d_relinvovl;i++){
            //create all phases for each window width
            d_windows[winsize][i].resize(winwidth, gr_complex( std::polar(1.0, 2.0 * M_PI * (double)i / (double)d_relinvovl ) ) );

            //create flank
            for(int k=0;k<puffersamples;k++){
                flank=0.5f - 0.5f*(float)cos(M_PI*(double)k/(double)puffersamples);
                d_windows[winsize][i][k]*=flank;
                d_windows[winsize][i][winwidth -1 -k]*=flank;
            }

        }

    }
}

void SegmentDetection_impl::save_hist(const gr_complex *in){
    if(d_hist.size()!=d_blocklen)
        d_hist.resize(d_blocklen, gr_complex(0.0f, 0.0f) );

    memcpy(d_hist.data(), in, d_blocklen*sizeof(gr_complex) );
}

void SegmentDetection_impl::set_chan_start_stop_width_dec(float start, float stop, float minchandist){
    //plausibility checks
    minchandist=mod_f(minchandist, 1.0f);
    start=mod_f(start, 1.0f);
    stop=mod_f(stop, 1.0f);

    if(start==stop)
        throw std::invalid_argument("Start must not be equal to stop. ");

    if(start>stop){
        // swapping start and stop.
        float tmp=start;
        start=stop;
        stop=tmp;
    }

    //set decimation factor by minimum channel distance
    double dec=(double)d_blocklen * (double)minchandist /2.0;

    if(dec<2.0)
        d_chan_detection_decimation_factor=1;
    else
        //chan_detection_decimation_factor=lastpow2(dec);
        d_chan_detection_decimation_factor=(int) dec;

    //determine width by decimation factor
    size_t width=(size_t) ((double)(stop-start) * (double) d_blocklen);
    if(width%d_chan_detection_decimation_factor)
        width+=d_chan_detection_decimation_factor - width%d_chan_detection_decimation_factor;
    if(width>d_blocklen) //this should not happen, but to be sure.
        width=d_blocklen - (d_blocklen%d_chan_detection_decimation_factor);

    d_width=width;

    //determine extraction start and stop
    size_t mid=(size_t) ((double) (0.5f*(start+stop)) * (double) d_blocklen);
    d_start=mid<d_width/2 ? 0 : mid-d_width/2; //start minimum at 0
    d_stop=d_start+d_width;
    if(d_stop > d_blocklen){
        d_stop=d_blocklen;
        d_start=d_stop-d_blocklen;
    }

    //reset power size.
    d_power.resize(d_width / d_chan_detection_decimation_factor);
}

void SegmentDetection_impl::set_threading(bool threading){
    d_threading=threading;

    if(d_threading)
        process_active_channels=&SegmentDetection_impl::process_active_channels_multi_thread;
    else
        process_active_channels=&SegmentDetection_impl::process_active_channels_single_thread;

}

void SegmentDetection_impl::fftshift(gr_complex *in, gr_complex *out, int sz){
    if(sz%2)
        sz--;

    int N2=sz/2;

    memcpy( out, in+N2, N2*sizeof(gr_complex) );
    memcpy( out+N2, in, N2*sizeof(gr_complex) );
}

void SegmentDetection_impl::log(str s){
    if(d_verbose==LOGTOCONSOLE)
        std::cout << s << std::endl;
    else if(d_verbose==LOGTOFILE){
        FILE *f=fopen(d_logfile.c_str(), "a");
        if(!f)
            std::cerr << "Outputfile not writable: " << d_logfile << std::endl;
        else{
            s+=str("\n");
            fwrite(s.c_str(), sizeof(char), s.size(), f);
        }
        fclose(f);
    }
}

str SegmentDetection_impl::get_ID_for_msg(size_t chanID){
    //convention for ID is timestamp.SRC.SEGMENTNUM.CONTNUM
    str s=get_current_time() + str(".DETECTED.") + num2str(d_ID) + str(".") + num2str(chanID);
    return s;
}

str SegmentDetection_impl::get_current_time(){
    time_t rawtime;
    struct tm * timeinfo;
    char p[40];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (p,80,"%Y-%m-%d-%H-%M-%S",timeinfo);

    str s(p);

    return s;
}


/*
 * Additional helping functions
 */

template <typename T>
float mod_f(T x, T y){
    return (float) fmod(fmod((double) x,(double) y)+1.0, (double) y);
}

template <typename T>
int nextpow2(T v){
    return 1<<(int) ceil(log2((double) v));
}

template <typename T>
int lastpow2(T v){
    return 1<<(int) floor(log2((double) v));
}

template <typename T>
bool ispow2(T v){
    if( v == (T) nextpow2(v) )
        return true;
    return false;
}


template <typename T>
str num2str(T v){
    std::ostringstream ss;
    ss << v;
    return str(ss.str());
}


} /* namespace FDC */
} /* namespace gr */

