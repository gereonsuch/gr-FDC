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
#include "PowerActivationChannel_impl.h"

namespace gr {
namespace FDC {

PowerActivationChannel::sptr
PowerActivationChannel::make(int v_blocklen, float v_cfreq, float v_bw, int v_relinvovl, float v_thresh, int v_maxblocks, int v_deactivation_delay, bool v_msg, bool v_fileoutput, std::string v_path, int verbose, int v_ID)
{
    return gnuradio::get_initial_sptr
            (new PowerActivationChannel_impl(v_blocklen, v_cfreq, v_bw, v_relinvovl, v_thresh, v_maxblocks, v_deactivation_delay, v_msg, v_fileoutput, v_path, verbose, v_ID));
}

/*
     * The private constructor
     */
PowerActivationChannel_impl::PowerActivationChannel_impl(int v_blocklen, float v_cfreq, float v_bw, int v_relinvovl, float v_thresh, int v_maxblocks, int v_deactivation_delay, bool v_msg, bool v_fileoutput, std::string v_path, int verbose, int v_ID)
    : gr::sync_block("PowerActivationChannel",
                     gr::io_signature::make(1, 1, sizeof(gr_complex) * v_blocklen),
                     gr::io_signature::make(0, 0, 0))
{
    //set channelizer ID
    ID=v_ID;

    //set debugging and logging output mode
    if(verbose==(int)LOGTOCONSOLE)
        verbosemode=LOGTOCONSOLE;
    else if(verbose==(int)LOGTOFILE){
        verbosemode=LOGTOFILE;
        logfile=str("gr-FDC.PowActChan.")+std::to_string(ID)+str(".")+str("log");
        FILE *f=fopen(logfile.c_str(), "w");
        if(!f)
            std::cerr << "Logfile not writable: " << logfile << std::endl;
        else
            fwrite("\n", sizeof(char), 1, f);
        fclose(f);
    }else
        verbosemode=NOLOG;

    //set signal paramters and check plausibility
    if(v_blocklen<=0)
        throw std::invalid_argument(str("Blocklen invalid, must be >0, is ")+std::to_string(v_blocklen));
    blocklen=v_blocklen;

    if(v_relinvovl<=0 || !ispow2(v_relinvovl))
        throw std::invalid_argument(str("Relinvovl invalid, must be >0 and power of 2, is ")+std::to_string(v_relinvovl));
    relinvovl=v_relinvovl;

    set_startstop(v_cfreq, v_bw);

    set_thresh(v_thresh);

    maxblocks=v_maxblocks; /* <0 => Never emit unfinished
                            * =0 => Directly emit unfinished
                            * >0 => emit after N blocks
                            */

    //deactivate signal after deactivation_delay consecutive blocks power under thresh.
    if(v_deactivation_delay<=0)
        deactivation_delay=0;
    else
        deactivation_delay=v_deactivation_delay;

    //init history
    hist.resize(blocklen, gr_complex(0.0f,0.0f));

    //lastpower is the power measured in the last block
    lastpower=std::numeric_limits<float>::max(); //init max to avoid initial activation

    active=false; //init inactive...

    blockcount=1; //hist is block 0

    //set output modi
    msg=v_msg;
    if(msg){
        msgport=pmt::intern("msgout");
        message_port_register_out(msgport);
    }

    fileoutput=v_fileoutput;
    if(fileoutput){
        path=v_path;
    }

    finished_channels=0;


    if(verbosemode){
        str s("############################\n\n");
        s+=str("# ")+str("gr-FDC.PowActChan.")+std::to_string(ID)+str("\n\n")+s+
                str("# extract_start: ") + std::to_string(extract_start) + str("\n")+
                str("# extract_stop: ") + std::to_string(extract_stop) + str("\n")+
                str("# extract_width: ") + std::to_string(extract_width) + str("\n")+
                str("# measure_start: ") + std::to_string(measure_start) + str("\n")+
                str("# measure_stop: ") + std::to_string(measure_stop) + str("\n\n")+
                str("# equivalent cfreq: ") + std::to_string( (double)(extract_start+extract_width/2)/(double)blocklen ) + str("\n")+
                str("# equivalent bw: ") + std::to_string( (double)extract_width/(double)blocklen ) + str("\n\n");
        log(s);
    }



}

/*
     * Our virtual destructor.
     */
PowerActivationChannel_impl::~PowerActivationChannel_impl()
{
}

int
PowerActivationChannel_impl::work(int noutput_items,
                                  gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];

    const gr_complex *sighist=hist.data();
    const gr_complex *sig=in;

    for(int i=0;i<noutput_items;i++){
        sig=in+i*blocklen; //current signal block is shifted by one block

        if(measure_power(sig)){
            //if true, state needs to be changed
            if(!active)
                activate(sighist, sig);
            else{
                process_channel(sig); //last block needs to be processed

                deactivate();
            }
        }else if(active){
            //if true, channel is active and signal needs to be processed
            process_channel(sig);

            if(maxblocks==0 || (maxblocks>0 && count%maxblocks==0))
                //if true, emit data before fin
                emit_data(false);
        }


        sighist=sig; //new history is the last processed block
        blockcount++;
    }

    save_hist(sig);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}





//functionality methods

void PowerActivationChannel_impl::save_hist(const gr_complex *sig){
    memcpy(hist.data(), sig, sizeof(gr_complex)*blocklen);
}

void PowerActivationChannel_impl::deactivate(){
    active=false;

    //emit all data
    emit_data();

    finished_channels++;
}

void PowerActivationChannel_impl::activate(const gr_complex *sighist, const gr_complex *sig){
    //activate new carrier
    part=0;
    count=0;
    active=true;
    phase=0;
    blocks.clear();
    msgID=create_ID();

    //process previous and current block and append to block deque
    process_channel(sighist);
    process_channel(sig);
}

void PowerActivationChannel_impl::emit_data(bool fin){
    std::vector<gr_complex> d;
    d.resize(blocks.size() * output_len); //output_len must not have changed. currently, this is not implemented

    int i=0;
    for(std::vector<gr_complex> &v: blocks)
        memcpy(d.data()+(i++)*output_len, v.data(), sizeof(gr_complex)*v.size());

    blocks.clear();

    if(msg){ //emit channel data to msgport
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern( msgID+(fin?str(".fin"):str(".part")) ));
        dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(fin));
        dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(part));
        dict = pmt::dict_add(dict, pmt::intern("rel_cfreq"), pmt::from_double( (double)(extract_start+extract_stop)/2.0/(double)blocklen ));
        dict = pmt::dict_add(dict, pmt::intern("rel_bw"), pmt::from_double( (double)extract_width/(double)blocklen ));
        dict = pmt::dict_add(dict, pmt::intern("blockstart"), pmt::from_long(blockcount-count));
        dict = pmt::dict_add(dict, pmt::intern("blockend"), pmt::from_long(blockcount));

        message_port_pub(msgport, pmt::cons(dict, pmt::init_c32vector( d.size(), d ))); //emit as PDU
    }

    if(fileoutput){ //write channel data to file
        str filename=path+str("/")+   //path
                msgID+(fin?str(".fin"):(str(".parted.")+std::to_string(part))); //ID-filename
        FILE* fh=fopen( filename.c_str(), "wb" );
        if(!fh)
            std::cerr << "Cannot write to file " << filename << std::endl; //dont want to throw errors here...
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), fh);
        fclose(fh);
    }

    if(verbosemode){
        std::string s=msgID+(fin?str(".fin"):(str(".parted.")+std::to_string(part)))+str(": ");
        s+=std::string("start=")+std::to_string(extract_start)+
                std::string(", stop=")+std::to_string(extract_stop)+
                std::string(", blockstart=")+std::to_string(blockcount-count)+
                std::string(", blockend=")+std::to_string(blockcount);
        log(s);
    }


    part++;

}

void PowerActivationChannel_impl::process_channel(const gr_complex *sig){
    std::vector< gr_complex > x; //tmp vector
    x.resize(extract_width);

    gr::fft::fft_complex ifft(extract_width, false, 1); //gnuradio fft object

    //multiply with window
    volk_32fc_x2_multiply_32fc(x.data(), sig+extract_start, windows[phase].data(), extract_width);

    //fftshift
    fftshift(x.data(), ifft.get_inbuf(), extract_width);

    //ifft
    ifft.execute();

    //copy ifft output buffer without overlap to data storeage puffer.
    blocks.push_back( {} );
    blocks.back().resize(output_len);

    memcpy(blocks.back().data(), ifft.get_outbuf()+output_ovl_offset, sizeof(gr_complex)*output_len);

    //increment counters.
    count++;
    phase=(phase+deltaphase)%relinvovl;
}

bool PowerActivationChannel_impl::measure_power(const gr_complex *in){
    //returns true if state has changed(either way)

    float pwr=0.0f;
    for(int i=measure_start;i<measure_stop;i++)
        pwr+=std::real( in[i] * std::conj(in[i]) );

    if(pwr==0.0f)
        pwr=std::numeric_limits<float>::min(); //avoid zero division

    if((!active) && pwr/lastpower>=thresh){
        lastpower=pwr;
        return true;
    }else if(active && lastpower/pwr>=thresh){
        lastpower=pwr;
        return true;
    }

    lastpower=pwr;
    return false;
}

std::string PowerActivationChannel_impl::create_ID(){
    //convention for ID is timestamp.SRC.SEGMENTNUM.CONTNUM
    str s=get_current_time() + str(".PowActChan.") + std::to_string(ID) + str(".") + std::to_string(finished_channels);
    return s;
}

void PowerActivationChannel_impl::set_startstop(float cfreq, float bw){
    bw=bw>0.0f?bw:-bw; //abs(bw)

    //plausibility
    if(bw>1.0 || cfreq-bw/2.0f<0.0f || cfreq+bw/2.0f>1.0f)
        throw std::invalid_argument(str("Desired channel is out of band: cfreq=")+std::to_string(cfreq)+str(", bw=")+std::to_string(bw));

    //set parameters and check plausibility
    extract_width=nextpow2((int) ceil((double)bw * (double)blocklen));
    if(extract_width>blocklen)
        extract_width=blocklen;

    int mid=(int) round( (double)cfreq * (double)blocklen );

    extract_start=mid-extract_width/2;
    if(extract_start<0)
        extract_start=0;

    extract_stop=extract_start+extract_width;
    if(extract_stop>blocklen){
        extract_stop=blocklen;
        extract_start=extract_stop-blocklen;
    }

    measure_start=(int) round((double)(cfreq-bw/2.0f) * (double)blocklen);
    measure_stop=(int) round((double)(cfreq+bw/2.0f) * (double)blocklen);

    //i dont see why this would happen.... but plausibility check
    if(measure_start<extract_start)
        measure_start=extract_start;
    if(measure_stop>extract_stop)
        measure_stop=extract_stop;

    cr_windows();

    deltaphase=extract_start%relinvovl;
    phase=0;

    output_ovl_offset=extract_width/relinvovl;
    output_len=extract_width-output_ovl_offset;

}

void PowerActivationChannel_impl::cr_windows(){
    int rampsamps=(extract_stop - extract_start) - (measure_stop - measure_start); //these are the out of channel samps
    rampsamps/=3; //1 third is left edge, 1 third is right edge, 1 third is puffer to avoid filtering the channel

    //cr rect windows with shifted phases
    windows.clear();
    windows.reserve(relinvovl);
    for(int i=0;i<relinvovl;i++)
        windows.push_back( std::vector<gr_complex>(blocklen, std::polar(1.0f, (float)(2.0f*M_PI*(double)i/(double)relinvovl) ) ) );

    //create bandpass rising edges for each filter
    for(int i=0;i<rampsamps;i++){
        for(std::vector<gr_complex> &v: windows){
            v[i]*=(float) sin(0.5*M_PI*(double)i/(double)(rampsamps+1));
            v[blocklen-i-1]=v[i];
        }
    }

}

void PowerActivationChannel_impl::set_thresh(float v_thresh){
    if(v_thresh<=0.0f)
        throw std::invalid_argument(str("Threshold is interpreted as dB and must be >0.0, is ")+std::to_string(v_thresh));
    thresh=(float) pow( 10.0, (double)v_thresh/10.0 );
}







//helping methods

bool PowerActivationChannel_impl::ispow2(int k){
    return k==nextpow2(k);
}

int PowerActivationChannel_impl::nextpow2(int k){
    if(k<=0){
        throw std::invalid_argument(str("Can't eval nextpow2 from ")+std::to_string(k)+str("\n"));
    }
    return (int) std::pow(2,ceil(log2((double) k)));
}

void PowerActivationChannel_impl::log(str s){
    if(verbosemode==LOGTOCONSOLE)
        std::cout << s << std::endl;
    else if(verbosemode==LOGTOFILE){
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

void PowerActivationChannel_impl::fftshift(const gr_complex *src, gr_complex *dest, int sz){
    if(sz%2)
        sz-=1;

    if(sz==0)
        return;
    if(sz<0)
        throw std::invalid_argument(str("FFTshift cannot be performed with size<0, is ")+std::to_string(sz));

    int sz2=sz/2;

    memcpy(dest, src+sz2, sizeof(gr_complex)*sz2);
    memcpy(dest+sz2, src, sizeof(gr_complex)*sz2);
}

str PowerActivationChannel_impl::get_current_time(){
    time_t rawtime;
    struct tm * timeinfo;
    char p[40];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (p,80,"%Y-%m-%d-%H-%M-%S",timeinfo);

    str s(p);

    return s;
}

} /* namespace FDC */
} /* namespace gr */

