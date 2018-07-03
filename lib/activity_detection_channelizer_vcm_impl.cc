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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "activity_detection_channelizer_vcm_impl.h"

namespace gr {
namespace FDC {

template <typename T>
std::string num2str(T k){
    std::ostringstream s;
    s << k;
    return s.str();
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

std::string get_current_time(){
    time_t rawtime;
    struct tm * timeinfo;
    char p[40];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (p,80,"%Y-%m-%d-%H-%M-%S",timeinfo);

    std::string s(p);

    return s;
}


activity_detection_channelizer_vcm::sptr
activity_detection_channelizer_vcm::make(int v_blocklen, std::vector< std::vector< float > > v_segments, float v_thresh, int v_relinvovl, int v_maxblocks, bool v_message, bool v_fileoutput, std::string v_path, bool v_threads, float v_minchandist, int v_channel_deactivation_delay, double v_window_flank_puffer, int verbose)
{
    return gnuradio::get_initial_sptr
            (new activity_detection_channelizer_vcm_impl(v_blocklen, v_segments, v_thresh, v_relinvovl, v_maxblocks, v_message, v_fileoutput, v_path, v_threads, v_minchandist, v_channel_deactivation_delay, v_window_flank_puffer, verbose));
}

/*
     * The private constructor
     */
activity_detection_channelizer_vcm_impl::activity_detection_channelizer_vcm_impl(int v_blocklen, std::vector< std::vector< float > > v_segments, float v_thresh, int v_relinvovl, int v_maxblocks, bool v_message, bool v_fileoutput, std::string v_path, bool v_threads, float v_minchandist, int v_channel_deactivation_delay, double v_window_flank_puffer, int v_verbose)
    : gr::sync_block("activity_detection_channelizer_vcm",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)*v_blocklen),
                     //gr::io_signature::make(1, 1, sizeof(float)*( v_blocklen<4096?v_blocklen:4096 )))
                     gr::io_signature::make(0, 0, 0 ))
{
    //init log(file)
    if(v_verbose==(int) LOGTOCONSOLE)
        verbose = LOGTOCONSOLE;
    else if(v_verbose==(int) LOGTOFILE){
        verbose = LOGTOFILE;
        //'gr-FDC.FreqDomChan.'+time.asctime().replace(' ','_')+'.log'
        logfile=std::string("gr-FDC.ActDetChan.log");
        FILE *f=fopen(logfile.c_str(), "w");
        if(!f)
            std::cerr << "Logfile not writable: " << logfile << std::endl;
        else
            fwrite("\n", sizeof(char), 1, f);
        fclose(f);
    }else
        verbose = NOLOG;


    //plausibility and setting up blocklen.
    if(v_blocklen<2 || !ispow2(v_blocklen))
        throw std::invalid_argument("Blocklen invalid. ");
    blocklen=v_blocklen;

    //adjust history puffer to blocklen
    hist.resize(blocklen, gr_complex(0.0f,0.0f) );

    //setting up a decimation factor for activity detection by minimum channel distance.
    set_chan_detection_decimation_factor(v_minchandist);

    //plausibility and setting up threshold
    if(v_thresh<0.0f)
        throw std::invalid_argument("Threshold is interpreted as dB and must be greater zero. ");
    thresh=(float) pow( 10.0, (double)v_thresh/10.0 );

    //plausibility and setting up relinvovl
    if(v_relinvovl<1 || !ispow2(v_relinvovl))
        throw std::invalid_argument("Relative inverse overlap is invalid, must be >0 and a power of 2. ");
    relinvovl=v_relinvovl;

    //setting up maxblocks
    maxblocks=v_maxblocks; /* >0  => after "maxblocks" blocks, they are emissioned.
                            * =0  => Each block is emissioned
                            * <0  => channel is only sent if finished correctly(might take forever and fill the memory)
                            */

    //setting up consecutive block deactivation delay
    channel_deactivation_delay=v_channel_deactivation_delay;
    if(channel_deactivation_delay<0)
        throw std::invalid_argument("Channel deactication delay must not be smaller 0. \n");

    //setting up flank puffer
    window_flank_puffer=v_window_flank_puffer;
    if(window_flank_puffer<0.0)
        throw std::invalid_argument("Window flank puffer must not be smaller 0.0. \n");

    //setting up message port
    msg=v_message;
    if(msg){
        outport=pmt::intern("msgout");
        message_port_register_out(outport);
    }

    //setting up fileoutput
    fileoutput=v_fileoutput;
    if(fileoutput){
        //maybe check plausibility?
        path=v_path;
    }

    //setting up threading
    threading=v_threads;

    //assigning specific threaded or singlethread methods to specific function pointers to avoid checking of threading attribute in each iteration in main loop
    if(threading){
        detect_channels_in_segments=&activity_detection_channelizer_vcm_impl::detect_channels_in_segments_threaded;
        extract_channels_in_segments=&activity_detection_channelizer_vcm_impl::extract_channels_in_segments_threaded;
    }else{
        detect_channels_in_segments=&activity_detection_channelizer_vcm_impl::detect_channels_in_segments_singlethread;
        extract_channels_in_segments=&activity_detection_channelizer_vcm_impl::extract_channels_in_segments_singlethread;
    }

    //setting up all segments.
    segments.reserve( v_segments.size() ); //memory allocation
    for(std::vector<float> &v: v_segments){
        create_segment(v);
    }

    cr_windows();

    if(verbose){
        for(segment &seg: segments){
            std::string s("# Segment ");
            s+=num2str(seg.ID) + std::string(": \n");
            s+=std::string("# start: ") + num2str(seg.start) + std::string(" => f_start=") + num2str((double)seg.start/(double)blocklen) + std::string("\n");
            s+=std::string("# stop: ") + num2str(seg.stop) + std::string(" => f_stop=") + num2str((double)seg.stop/(double)blocklen) + std::string("\n");
            s+=std::string("# width: ") + num2str(seg.width) + std::string(" => f_bw=") + num2str((double)seg.width/(double)blocklen) + std::string("\n");
            s+=std::string("# chan_decimation_fact: ") + num2str(seg.chan_detection_decimation_factor) + std::string("\n");
            log(s);
        }
    }

    blockcount=1;

}

/*
     * Our virtual destructor.
     */
activity_detection_channelizer_vcm_impl::~activity_detection_channelizer_vcm_impl()
{
}

void activity_detection_channelizer_vcm_impl::cr_windows(){
    int num_window_sizes=(int) log2((double) blocklen)+1;
    int winwidth;
    int puffersamples;
    float flank;

    windows.resize(num_window_sizes);

    for(int winsize=0;winsize<num_window_sizes;winsize++){
        windows[winsize].resize(relinvovl);

        winwidth=1<<winsize;

        puffersamples=(int)(window_flank_puffer*(double)winwidth);

        for(int i=0;i<relinvovl;i++){
            //create all phases for each window width
            windows[winsize][i].resize(winwidth, gr_complex( std::polar(1.0, 2.0 * M_PI * (double)i / (double)relinvovl ) ) );

            //create flank
            for(int k=0;k<puffersamples;k++){
                flank=0.5f - 0.5f*(float)cos(M_PI*(double)k/(double)puffersamples);
                windows[winsize][i][k]*=flank;
                windows[winsize][i][winwidth -1 -k]*=flank;
            }

        }

    }
}

void activity_detection_channelizer_vcm_impl::set_chan_detection_decimation_factor(float minchandist){
    if(minchandist<=0.0f || minchandist>=1.0)
        throw std::invalid_argument(std::string("Minimum channel distance is invalid. Must be in (0,1), is ")+num2str(minchandist));

    double dec=(double)blocklen * (double)minchandist /2.0;

    if(dec<2.0)
        chan_detection_decimation_factor=1;
    else
        //chan_detection_decimation_factor=lastpow2(dec);
        chan_detection_decimation_factor=(int) dec;

    /*if(blocklen%chan_detection_decimation_factor)
        throw std::invalid_argument("Error: blocklen is incompatible for power decimation factor. ");*/

    power.resize(blocklen / chan_detection_decimation_factor);
}

void activity_detection_channelizer_vcm_impl::create_segment(std::vector<float> &v){
    if(v.size()!=2 || v[0]>=v[1] || v[0]<0.0f || v[1]>1.0f){
        std::string s="Segment is incorrect. must be of size 2 with each member in (0,1), with v[0]<v[1]. v is [";
        for(float k: v)
            s+=num2str(k) + std::string(", ");
        s+=std::string("]");
        throw std::invalid_argument(s);
    }


    int mid=(int) abs(round(((double) v[1] + (double)v[0])*0.5*(double)blocklen) );
    int width=(int) abs( round(((double) v[1] - (double)v[0])*(double)blocklen) );
    width=(width%chan_detection_decimation_factor==0)?width:width+chan_detection_decimation_factor-width%chan_detection_decimation_factor;
    while(width>=blocklen)
        width=blocklen - (blocklen%chan_detection_decimation_factor);


    int start=mid-width/2 <= 0? 0 : mid-width/2;
    int stop=start+width;
    if(stop>blocklen){
        stop=blocklen;
        start=blocklen-width;
    }

    if(start < 0 || stop > blocklen){
        std::string s=std::string("Cannot evaluate start and stop of segment... start=")+num2str(start)+std::string(", stop=")+num2str(stop);
        throw std::invalid_argument(s);
    }

    //segments.size as ID.
    segments.push_back( segment(segments.size(), start, stop, blocklen, chan_detection_decimation_factor, relinvovl, window_flank_puffer) );
}

void activity_detection_channelizer_vcm_impl::save_hist(const gr_complex *in){
    if(hist.size()!=blocklen)
        hist.resize(blocklen, gr_complex(0.0f, 0.0f) );

    memcpy(hist.data(), in, blocklen*sizeof(gr_complex) );
}

void activity_detection_channelizer_vcm_impl::detect_channels_in_segments_singlethread(const gr_complex *sig){
    for(segment &seg: segments)
        seg.detect_channels( sig, thresh );
}

void activity_detection_channelizer_vcm_impl::detect_channels_in_segments_threaded(const gr_complex *sig){
    std::vector<std::thread> threads;
    threads.reserve(segments.size());

    //start all detection threads
    for(segment &seg: segments)
        threads.push_back( std::thread(&segment::detect_channels, &seg, sig, thresh) );

    //wait for all threads to finish
    for(std::thread &p: threads)
        p.join();
}

void activity_detection_channelizer_vcm_impl::extract_channels_in_segments_singlethread(const gr_complex *lastblock, const gr_complex *curblock){
    if(maxblocks>=0){
        for(segment &seg:segments){
            for(struct active_channel &c: seg.active_channels){
                if(c.inactive < 0)
                    process_channel_hist(lastblock, curblock, c);
                else if(c.inactive > channel_deactivation_delay)
                    emit_channel(c, seg); //channel is emitted but not deleted. see clear_inactive_channels()
                else
                    process_channel(curblock, c);

                if(c.data.size()>=maxblocks)
                    emit_unfinished_channel(c, seg);

            }
        }
    }else{
        for(segment &seg:segments){
            for(struct active_channel &c: seg.active_channels){
                if(c.inactive < 0)
                    process_channel_hist(lastblock, curblock, c);
                else if(c.inactive > channel_deactivation_delay)
                    emit_channel(c, seg); //channel is emitted but not deleted. see clear_inactive_channels()
                else
                    process_channel(curblock, c);
            }
        }
    }

    //erase all inactive channels from all segments
    clear_inactive_channels();
}

void activity_detection_channelizer_vcm_impl::extract_channels_in_segments_threaded(const gr_complex *lastblock, const gr_complex *curblock){
    std::deque< std::thread > threads;

    for(segment &seg: segments){
        for(struct active_channel &c: seg.active_channels){
            if(c.inactive < 0)
                threads.push_back( std::thread( &activity_detection_channelizer_vcm_impl::process_channel_hist, this, lastblock, curblock, std::ref(c) ) );
            else if(c.inactive > channel_deactivation_delay)
                threads.push_back( std::thread( &activity_detection_channelizer_vcm_impl::emit_channel, this, std::ref(c), std::ref(seg) ) ); //channel is emitted but not deleted. see clear_inactive_channels()
            else
                threads.push_back( std::thread( &activity_detection_channelizer_vcm_impl::process_channel, this, curblock, std::ref(c) ) );
        }
    }


    for(std::thread &t: threads)
        t.join();
    threads.clear();

    //if one channel was active longer than specified maxblocks, emit it partly.
    //To avoid conflicts, this is done after finishing all dsp and regular emitting tasks

    if(maxblocks>=0)
        for(segment &seg: segments)
            for(struct active_channel &c: seg.active_channels)
                if(c.data.size()>=maxblocks)
                    threads.push_back( std::thread( &activity_detection_channelizer_vcm_impl::emit_unfinished_channel, this, std::ref(c), std::ref(seg) ) );

    for(std::thread &t: threads)
        t.join();

    clear_inactive_channels();
}

void activity_detection_channelizer_vcm_impl::process_channel(const gr_complex *sig, active_channel &c){
    std::vector< gr_complex > x; //tmp vector
    x.resize(c.extract_width);

    gr::fft::fft_complex ifft(c.extract_width, false, 1); //gnuradio fft object

    //multiply with window
    volk_32fc_x2_multiply_32fc(x.data(), sig+c.extract_start, windows[c.extract_window][c.phase].data()  , c.extract_width);

    //fftshift
    fftshift(x.data(), ifft.get_inbuf(), c.extract_width);

    //ifft
    ifft.execute();


    //copy ifft output buffer without overlap to data storeage puffer.
    c.data.push_back( {} );
    c.data.back().resize(c.outputsamples);
    memcpy(c.data.back().data(), ifft.get_outbuf()+c.ovlskip, c.outputsamples*sizeof(gr_complex));

    //increment counters.
    c.count++;
    c.phase=(c.phase+c.phaseincrement)%relinvovl;
}

void activity_detection_channelizer_vcm_impl::process_channel_hist(const gr_complex *hist_sig, const gr_complex *sig, active_channel &c){
    process_channel(hist_sig, c);
    process_channel(sig, c);
    c.inactive=0;
}


void activity_detection_channelizer_vcm_impl::emit_channel(active_channel &c, segment &seg){
    std::vector< gr_complex > d; //puffer vector.
    d.resize( c.data.size()*c.outputsamples );

    for(int i=0;i<c.data.size();i++) //could also emit all... but maybe following blocks need a fixed length invoming vector if parted.
        memcpy( d.data()+i*c.outputsamples, (c.data.begin()+i)->data(), c.outputsamples*sizeof(gr_complex) );

    c.data.clear();

    if(msg){ //emit channel data to msgport
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern(c.msg_ID));
        dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(true));
        if(c.part>0)
            dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(c.part)); //add part information if emitted previously
        //add samprate, carrierfrequency, sps, ... if known.
        dict = pmt::dict_add(dict, pmt::intern("rel_bw"), pmt::from_double((double)(c.extract_width)/(double)blocklen));
        dict = pmt::dict_add(dict, pmt::intern("rel_cfreq"), pmt::from_double((double)(c.extract_start+c.extract_stop)/2.0/(double)blocklen));
        dict = pmt::dict_add(dict, pmt::intern("blockstart"), pmt::from_long(blockcount-c.count));
        dict = pmt::dict_add(dict, pmt::intern("blockend"), pmt::from_long(blockcount));
        dict = pmt::dict_add(dict, pmt::intern("vectorstart"), pmt::from_long(c.extract_start));
        dict = pmt::dict_add(dict, pmt::intern("vectorend"), pmt::from_long(c.extract_stop));

        message_port_pub(outport, pmt::cons(dict, pmt::init_c32vector( d.size(), d ))); //emit as PDU
    }
    if(fileoutput){ //write channel data to file
        std::string filename=path+std::string("/")+c.msg_ID+std::string(".fin");
        FILE* fh=fopen( filename.c_str(), "wb" );
        if(!fh)
            std::cerr << "Cannot write to file " << filename << std::endl; //dont want to throw errors here...
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), fh);
        fclose(fh);
    }

    //channel is erased in other method.

    if(verbose){
        std::string s=c.msg_ID+std::string(".fin: ");
        s+=std::string("start=")+num2str(c.extract_start)+
                std::string(", stop=")+num2str(c.extract_stop)+
                std::string(", blockstart=")+num2str(blockcount-c.count)+
                std::string(", blockend=")+num2str(blockcount);
        log(s);
    }

}

void activity_detection_channelizer_vcm_impl::emit_unfinished_channel(active_channel &c, segment &seg){
    if(maxblocks<0 || c.data.size()<maxblocks) //in this case, this method should not be called. but nevertheless check it.
        return;

    //emit maxblocks or everything if maxblocks==0
    int Ntxblocks=maxblocks==0?c.data.size():maxblocks;

    if(Ntxblocks<=0)
        return; //just to be sure...

    std::vector< gr_complex > d; //puffer vector.
    d.resize( Ntxblocks*c.outputsamples );

    for(int i=0;i<Ntxblocks;i++)
        memcpy( d.data()+i*c.outputsamples, (c.data.begin()+i)->data(), c.outputsamples*sizeof(gr_complex) );

    c.data.erase(c.data.begin(), c.data.begin()+Ntxblocks); //erase all copied vectors.

    if(msg){ //emit channel data to msgport
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern(c.msg_ID));
        dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(false));
        dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(c.part));
        //add samprate, carrierfrequency, sps, ... if known.
        dict = pmt::dict_add(dict, pmt::intern("rel_bw"), pmt::from_double((double)(c.extract_width)/(double)blocklen));
        dict = pmt::dict_add(dict, pmt::intern("rel_cfreq"), pmt::from_double((double)(c.extract_start+c.extract_stop)/2.0/(double)blocklen));
        dict = pmt::dict_add(dict, pmt::intern("blockstart"), pmt::from_long(blockcount-c.count));
        dict = pmt::dict_add(dict, pmt::intern("blockend"), pmt::from_long(blockcount));
        dict = pmt::dict_add(dict, pmt::intern("vectorstart"), pmt::from_long(c.extract_start));
        dict = pmt::dict_add(dict, pmt::intern("vectorend"), pmt::from_long(c.extract_stop));

        message_port_pub(outport, pmt::cons(dict, pmt::init_c32vector( d.size(), d ))); //emit as PDU

    }
    if(fileoutput){ //write channel data to file
        std::string filename=path+std::string("/")+c.msg_ID+std::string(".parted.")+std::to_string(c.part);
        FILE* fh=fopen( filename.c_str(), "wb" );
        if(!fh)
            std::cerr << "Cannot write to file " << filename << std::endl; //dont want to throw errors here...
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), fh);
        fclose(fh);
    }

    c.part++;

    if(verbose){
        std::string s=c.msg_ID+std::string(".part: ");
        s+=std::string("start=")+num2str(c.extract_start)+
                std::string(", stop=")+num2str(c.extract_stop)+
                std::string(", part=")+num2str(c.part)+
                std::string(", blockstart=")+num2str(blockcount-c.count)+
                std::string(", blockend=")+num2str(blockcount);
        log(s);
    }

}

void activity_detection_channelizer_vcm_impl::clear_inactive_channels(){
    int i;

    for(segment &seg: segments){
        i=0;
        while(i<seg.active_channels.size()){
            if( (seg.active_channels.begin()+i)->inactive > channel_deactivation_delay ){
                seg.active_channels.erase( seg.active_channels.begin()+i );
            }else
                i++;
        }
    }
}

std::string get_ID_for_msg(int chanID, int segID){
    //convention for ID is timestamp.SRC.SEGMENTNUM.CONTNUM
    std::string ID=get_current_time() + std::string(".DETECTED.") + std::to_string(segID) + std::string(".") + std::to_string(chanID);
    return ID;
}

void activity_detection_channelizer_vcm_impl::fftshift(gr_complex *in, gr_complex *out, int sz){
    if(sz%2)
        sz--;

    int N2=sz/2;

    memcpy( out, in+N2, N2*sizeof(gr_complex) );
    memcpy( out+N2, in, N2*sizeof(gr_complex) );
}

int
activity_detection_channelizer_vcm_impl::work(int noutput_items,
                                              gr_vector_const_void_star &input_items,
                                              gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];
    gr_complex *out = (gr_complex *) output_items[0];

    const gr_complex *sig;
    const gr_complex *sig_hist = hist.data(); //pointer to last block, initialized with saved history.

    for(int i=0;i<noutput_items;i++){
        sig=in+i*blocklen;

        //detect channels in all segments. method pointer to threaded or singlethread
        //method is set in constructor.
        (this->*(detect_channels_in_segments))(sig);

        //extract all activated channels from current block
        //method is set in constructor
        (this->*(extract_channels_in_segments))(sig_hist, sig);


        //finally iterate history pointer
        sig_hist=sig;

        blockcount++;
    }

    //saving last block to history puffer
    save_hist(sig);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

void activity_detection_channelizer_vcm_impl::log(std::string &s){
    if(verbose==LOGTOCONSOLE)
        std::cout << s << std::endl;
    else if(verbose==LOGTOFILE){
        FILE *f=fopen(logfile.c_str(), "a");
        if(!f)
            std::cerr << "Outputfile not writable: " << logfile << std::endl;
        else{
            s+=std::string("\n");
            fwrite(s.c_str(), sizeof(char), s.size(), f);
        }
        fclose(f);
    }
}


/*
 * SEGMENT CLASS
 */

segment::segment(int v_ID, int v_start, int v_stop, int v_blocklen, int v_chan_detection_decimation_factor, int v_relinvovl, double v_window_flank_puffer){
    ID=v_ID;
    active_channels_counter=0;
    start=v_start;
    stop=v_stop;
    width=stop-start;
    chan_detection_decimation_factor=v_chan_detection_decimation_factor;
    blocklen=v_blocklen;
    relinvovl=v_relinvovl;
    window_flank_puffer=v_window_flank_puffer;

    if(width%chan_detection_decimation_factor)
        throw std::invalid_argument(std::string("Invalid segment width. Not a multiple of channel detection decimation factor. width=") +
                                    num2str(width) + std::string(", chan_det_dec_fact=") + num2str(chan_detection_decimation_factor));

    power.resize(width/chan_detection_decimation_factor);
}


void segment::detect_channels(const gr_complex *in, float thresh){
    //measure current power
    measure_power(in);

    std::deque< std::array<int,2> > poss_chans;

    //detect all possible channels, wether or not already active
    get_active_channels(poss_chans, thresh);

    //compare with active channels, activate and deactivate them.
    match_active_channels(poss_chans);
}

void segment::measure_power(const gr_complex *in){
    int N=width / chan_detection_decimation_factor;
    int L;
    float normfact=1.0f / (float) chan_detection_decimation_factor;
    std::vector<float> tmppower;
    tmppower.resize(N, 0.0f);


    if(power.size()!=N)
        power.resize(N, 0.0f);

    for(int i=0;i<N;i++){
        L=start + i*chan_detection_decimation_factor;

        for(int k=0;k<chan_detection_decimation_factor;k++)
            tmppower[i]+=std::real( in[L+k] * std::conj(in[L+k]) );
    }

    volk_32f_s32f_multiply_32f( power.data(), tmppower.data(), normfact, N );

}

/*void segment::get_active_channels(std::deque< std::array<int,2> > &poss_chans, float thresh){
    bool active=false;
    int poss_start;
    float powerdiff;

    float inversethresh=1.0f / thresh;

    for(int i=1;i<power.size();i++){

        if(power[i-1] == 0.0f)
            powerdiff=power[i] / std::numeric_limits<float>::min();
        else
            powerdiff=power[i] / power[i-1];

        if(!active && powerdiff > thresh){
            poss_start=(i-1)*chan_detection_decimation_factor + start;
            active=true;
        }else if(active && powerdiff < inversethresh){
            poss_chans.push_back( {poss_start, i*chan_detection_decimation_factor + start} );

            active=false;
        }
    }

}*/

int get_next_int(int a, std::deque<int> &lst){
    //lst is sorted!
    if(lst.size()==0)
        return -1;

    if(lst.front() > a)
        return lst.front();

    for(int k:lst)
        if(k > a)
            return k;

    //no next bigger int found.
    return -1;
}

void segment::get_active_channels(std::deque<std::array<int, 2> > &poss_chans, float thresh){
    std::deque< fipair > riseedge;
    std::deque< int > falledge; //no priorisation

    float inversethresh=1.0f / thresh;
    float powerdiff;

    //calculate all powerdifferences and get possible rising and falling edges
    for(int i=1;i<power.size();i++){
        if(power[i-1] == 0.0f)
            powerdiff=power[i] / std::numeric_limits<float>::min();
        else
            powerdiff=power[i] / power[i-1];

        if(powerdiff > thresh) riseedge.push_back( {powerdiff, (i-1)*chan_detection_decimation_factor + start} );
        if(powerdiff < inversethresh) falledge.push_back( i*chan_detection_decimation_factor + start );
    }

    //sort rising edges
    std::sort(riseedge.begin(), riseedge.end(), fipair_sort);

    int poss_start;
    int next_end;
    bool breaking=false;
    while(riseedge.size()){
        poss_start=riseedge.front().second;
        riseedge.pop_front();
        next_end=get_next_int(poss_start, falledge);

        if(next_end<=poss_start)
            continue; //discard if none found

        breaking=false;
        for(std::array<int, 2> &arr: poss_chans)
            if( poss_start<arr[1] && next_end>=arr[0] ){
                breaking=true;
                break; //if overlapping with any, discard
            }

        if(breaking)
            continue;

        //no check failed, add it to poss_chans
        poss_chans.push_back( {poss_start, next_end} );
    }
}

void segment::match_active_channels(std::deque< std::array<int,2> > &poss_chans){
    if(poss_chans.empty()){
        for(struct active_channel &c:active_channels)
            c.inactive+=1;
        return; //nothing to do since poss_chans is empty
    }

    int pc_start, pc_end;
    bool inactive;
    int i;

    for(struct active_channel &c: active_channels){
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
        for(std::array<int,2> &pc: poss_chans)
            if( !activate(pc[0], pc[1]) ){
                //Error occured, give additional infos
                std::cerr << "Possible Chans[ " << poss_chans.size() <<" ]: ";
                for(std::array<int, 2> &arr: poss_chans)
                    std::cerr << "[" << arr[0] << ", " << arr[1] << "], ";
                std::cerr << std::endl;
            }
}

bool segment::activate(int detect_start, int detect_end){
    //Creates a new channel in the segment. Start and stop of channel may exceed the segment, since
    //the extraction flank exceeds the detection bandwidth.

    int detect_width=detect_end - detect_start;
    int extract_mid=detect_start + detect_width/2;
    int extract_width=nextpow2( (int) ceil((double)detect_width * (1.0+2.0*window_flank_puffer)) );

    if(extract_width>blocklen){
        std::string s=std::string("channel width exceeds blocklen. start=") + num2str(detect_start) +
                std::string(", end=") + num2str(detect_end) +
                std::string(", width=")+ num2str(detect_width)+
                std::string("\nresulting in width=")+num2str(extract_width) +
                std::string(" blocklen=")+num2str(blocklen)+std::string("\n\n");
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
    if(extract_end>blocklen){
        extract_end=blocklen;
        extract_start=blocklen-extract_width;
    }

    struct active_channel c;
    c.ID=active_channels_counter++; //increment counter and take last num as ID for new channel.
    c.detect_start=detect_start;
    c.detect_stop=detect_end;
    c.extract_start=extract_start;
    c.extract_stop=extract_end;
    c.extract_width=extract_width;
    c.extract_window=(int) log2((double) extract_width);
    c.ovlskip=extract_width / relinvovl;
    c.outputsamples=c.extract_width-c.ovlskip;
    c.count=0;
    c.phase=0;
    c.phaseincrement=extract_start%relinvovl;
    c.inactive=-1; //-1 is initialized.
    c.part=0;
    c.msg_ID=get_ID_for_msg(c.ID, ID);

    c.data.clear();

    active_channels.push_back( c );

    //std::cout << "Seg " << ID << ":\t activated channel " << c.ID << " \t " << detect_start << ", " << detect_end << ", " << extract_start << std::endl;

    return true;
}






} /* namespace FDC */
} /* namespace gr */

