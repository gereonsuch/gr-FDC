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
#include "activity_controlled_channelizer_vcm_impl.h"

namespace gr {
namespace FDC {


activity_controlled_channelizer_vcm::sptr
activity_controlled_channelizer_vcm::make(int blocklen, std::vector< std::vector< float > > channels, float thresh, int relinvovl, int maxblocks, bool message, bool fileoutput, std::string path, bool threaded, int verbose)
{
    return gnuradio::get_initial_sptr
            (new activity_controlled_channelizer_vcm_impl(blocklen, channels, thresh, relinvovl, maxblocks, message, fileoutput, path, threaded, verbose));
}



channel::channel(int id, int begin, int width, int red_width, int relovl){
    channelcount=0;
    start=begin;
    size=width;
    red_size=red_width;
    ovlskip=size-red_size;
    relinvovl=relovl;
    count=0;
    part=0;
    ID=id;

    active=false;
    curpwr=std::numeric_limits<float>::max(); //init max to avoid initial activation

    cr_window(size, 1.0f - (float)((bwpuffer-1.0)/2.5), 1.0f, windows, relinvovl, start%relinvovl, true);
}

bool channel::set_power(const gr_complex *v, double thresh){
    double pwr=0.0;
    for(int i=0;i<size;i++){
        pwr+=(double) std::real( v[i] * std::conj(v[i]) );
    }
    pwr/=(double) size;

    bool retval=false;

    if(active && curpwr > pwr * thresh ){
        deactivate();
        retval=true;
    }else if(!active && curpwr * thresh < pwr ){
        activate();
        retval=true;
    }

    curpwr=pwr;
    return retval;
}

void channel::activate(){
    active=true;
    blocks.clear();
    count=0;
    part=0;
    msgID=get_msg_ID();
}

void channel::deactivate(){
    active=false;
    channelcount++;
}

void channel::push_data(std::vector<gr_complex> &v){
    if(red_size != v.size())
        throw std::invalid_argument(std::string("Invalid channel block size: ") + std::to_string(v.size()) + std::string("\n"));
    blocks.push_back( v );
    count++;
}

void channel::push_data(std::vector< std::vector< gr_complex > > &v){
    for(int i=0;i<v.size();i++){
        push_data( v[i] );
        count++;
    }
}

void channel::get_data(std::vector<gr_complex> &d){
    d.resize( blocks.size() * red_size );

    int pos=0;
    for(std::vector<gr_complex> &v: blocks){
        for(int i=0;i<v.size();i++)
            d[pos+i]=v[i];
        pos+=v.size();
    }
}


std::string channel::get_msg_ID(){
    //convention for ID is timestamp.SRC.CHANNELID.CONTNUM
    std::string s=get_time() + std::string(".ACTIVATED.") + std::to_string(ID) + std::string(".") + std::to_string(channelcount);
    return s;
}


/*
     * The private constructor
     */
activity_controlled_channelizer_vcm_impl::activity_controlled_channelizer_vcm_impl(int blocklen, std::vector< std::vector< float > > channels, float thresh, int relinvovl, int maxblocks, bool message, bool fileoutput, std::string path, bool threaded, int v_verbose)
    : gr::sync_block("activity_controlled_channelizer_vcm",
                     gr::io_signature::make(1, 1, sizeof(gr_complex)*blocklen),
                     //gr::io_signature::make(1, 1, sizeof(gr_complex)))
                     gr::io_signature::make(0, 0, 0))
{
    if(v_verbose==(int) LOGTOCONSOLE)
        verbose = LOGTOCONSOLE;
    else if(v_verbose==(int) LOGTOFILE){
        verbose = LOGTOFILE;
        //'gr-FDC.FreqDomChan.'+time.asctime().replace(' ','_')+'.log'
        logfile=std::string("gr-FDC.ActContrChan.log");
        FILE *f=fopen(logfile.c_str(), "a");
        if(!f)
            std::cerr << "Logfile not writable: " << logfile << std::endl;
        else
            fwrite("\n", sizeof(char), 1, f);
        fclose(f);
    }else
        verbose = NOLOG;

    veclen = blocklen;
    if(veclen<=0)
        throw std::invalid_argument(std::string("Invalid blocklen: ")+std::to_string(veclen)+std::string("\n"));

    threshold = pow(10.0,thresh/10.0);

    relovl = relinvovl;
    if(veclen<=0)
        throw std::invalid_argument(std::string("Invalid relinvovl: ")+std::to_string(relovl)+std::string("\n"));

    maximumblocks = maxblocks;
    if(veclen<=0)
        throw std::invalid_argument(std::string("Invalid maximum blocks: ")+std::to_string(maximumblocks)+std::string("\n"));

    msg = message;
    fileout = fileoutput;

    outputpath = path;

    chans.reserve(channels.size());
    channelcounter=0;
    for(int i=0;i<channels.size();i++){
        if(channels[i].size()!=2)
            throw std::invalid_argument(std::string("Invalid channel size for channel ")+std::to_string(i)+std::string(": ")+std::to_string(channels[i].size())+std::string("\n"));

        //channels[i][0] is normalized carrier frequency, channels[i][1] is normalized carrier bandwidth
        int start = (int)( round((channels[i][0] - channels[i][1]/2.0) * (double)veclen) );
        int width = nextpow2( (int)(round(channels[i][1] * (double)veclen * bwpuffer)) );

        if(width<=0 || width>veclen)
            throw std::invalid_argument(std::string("Invalid channel bandwidth for channel ")+std::to_string(i)+std::string(": ")+std::to_string(width)+std::string("\n"));

        //adjusting start to not overlap end.
        if(start<0)
            start=0;
        else if(start+width>veclen)
            start=veclen-width;

        if(start<0 || start+width>veclen)
            throw std::invalid_argument(std::string("Invalid channel start for channel ")+std::to_string(i)+std::string(": ")+std::to_string(start)+std::string("\n"));




        chans.push_back(channel(channelcounter++, start, width, width - width/relovl, relovl));

        std::cout << "# Init Chan " << chans[chans.size()-1].ID << ": f=" << chans[chans.size()-1].start << ", size=" << chans[chans.size()-1].size << std::endl;
    }

    hist.resize(veclen, gr_complex(0.0,0.0));

    msgport=pmt::intern("msgout");

    message_port_register_out(msgport);

    if(threaded)
        check_channels = &activity_controlled_channelizer_vcm_impl::check_channels_multithread;
    else
        check_channels = &activity_controlled_channelizer_vcm_impl::check_channels_singlethread;
}

/*
     * Our virtual destructor.
     */
activity_controlled_channelizer_vcm_impl::~activity_controlled_channelizer_vcm_impl()
{
}

int
activity_controlled_channelizer_vcm_impl::work(int noutput_items,
                                               gr_vector_const_void_star &input_items,
                                               gr_vector_void_star &output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];

    //first iter with hist
    (this->*(check_channels))(hist.data(), in);

    //continue with every previous block from input
    for(int i=1;i<noutput_items;i++){
        (this->*(check_channels))(in + (i-1)*veclen, in + i*veclen);
    }


    //save last block for next iteration
    memcpy(hist.data(), in+(noutput_items-1)*veclen, veclen*sizeof(gr_complex));

    return noutput_items;
}

void activity_controlled_channelizer_vcm_impl::check_channels_singlethread(const std::complex<float> *lastblock, const std::complex<float> *curblock){
    for(channel &c: chans){
        if(c.set_power(curblock + c.start, threshold)){
            //true if state changed(active<->inactive)
            if(c.active){
                std::cout << "# Chan " << c.ID << "activated" << std::endl;
                //just activated
                get_channel_data_hist(lastblock+c.start, curblock+c.start, c);
            }else{
                std::cout << "# Chan " << c.ID << "deactivated" << std::endl;
                //just deactivated
                tx_data(c);
            }
        }else if(c.active){
            //state has not changed and channel is active
            if(c.blocks.size() >= maximumblocks){
                //emit data and continue appending. this is necessary for continous channels
                tx_data(c); // DO NOT THREAD HERE!
                // appending of next block might lead to conflicts in channel object
            }

            //appending current data
            get_channel_data(curblock+c.start, c);
        }

    }
}

void activity_controlled_channelizer_vcm_impl::check_channels_multithread(const std::complex<float> *lastblock, const std::complex<float> *curblock){
    for(channel &c: chans){
        if(c.set_power(curblock + c.start, threshold)){
            //true if state changed(active<->inactive)
            if(c.active){
                std::cout << "# Chan " << c.ID << "activated" << std::endl;
                //just activated
                threads.push_back( std::thread(get_channel_data_hist, lastblock+c.start, curblock+c.start, std::ref(c)) );
            }else{
                std::cout << "# Chan " << c.ID << "deactivated" << std::endl;
                //just deactivated
                threads.push_back( std::thread(&activity_controlled_channelizer_vcm_impl::tx_data, this, std::ref(c)) );
            }
        }else if(c.active){
            //state has not changed and channel is active
            if(c.blocks.size() >= maximumblocks){
                //emit data and continue appending. this is necessary for continous channels
                tx_data(c); // DO NOT THREAD HERE!
                // appending of next block might lead to conflicts in channel object
            }

            //appending current data
            threads.push_back( std::thread(get_channel_data, curblock+c.start, std::ref(c)) );
        }

    }

    //wait for all channels to finish
    joinall();

    threads.clear();
}

void activity_controlled_channelizer_vcm_impl::joinall(){
    for(std::thread &t: threads)
        t.join();
}

void activity_controlled_channelizer_vcm_impl::tx_data(channel &c){
    std::vector<gr_complex> d;
    c.get_data(d);
    c.blocks.clear();

    if(msg){
        pmt::pmt_t dict = pmt::make_dict();
        dict = pmt::dict_add(dict, pmt::intern("ID"), pmt::intern(c.msgID));
        if(c.active){
            dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(false));
            dict = pmt::dict_add(dict, pmt::intern("part"), pmt::from_long(c.part));
        }else
            dict = pmt::dict_add(dict, pmt::intern("finalized"), pmt::from_bool(true));
        dict = pmt::dict_add(dict, pmt::intern("data"), pmt::init_c32vector( d.size(), d ));


        //tx message
        message_port_pub(msgport, dict);
    }

    if(fileout){
        std::string path=outputpath + std::string("/")+c.msgID;
        if(c.active)
            path+=std::string(".parted.")+std::to_string(c.part);
        else
            path+=std::string(".fin");

        FILE *f=fopen(path.c_str(), "wb");
        if(!f)
            std::cerr << "Outputfile not writable: " << path << std::endl;
        else
            fwrite(d.data(), sizeof(gr_complex), d.size(), f);
        fclose(f);
    }


    //update channel data
    c.part++;
}

void activity_controlled_channelizer_vcm_impl::log(std::string &s){
    if(verbose==LOGTOCONSOLE)
        std::cout << s << std::endl;
    else if(verbose==LOGTOFILE){
        FILE *f=fopen(logfile.c_str(), "a");
        if(!f)
            std::cerr << "Outputfile not writable: " << logfile << std::endl;
        else
            fwrite(s.c_str(), sizeof(char), s.size(), f);
        fclose(f);
    }
}




void get_channel_data(const gr_complex *v, channel &chan){
    std::vector<gr_complex> d;
    d.resize(chan.size);

    gr::fft::fft_complex ifft(chan.size, false, 1);


    //multiply window, d contains result
    volk_32fc_x2_multiply_32fc(d.data(), v, chan.windows[chan.count%chan.relinvovl].data() , chan.size);

    //fftshift
    fftshift(d.data(), ifft.get_inbuf(), chan.size);


    //inverse fast fourier transformation
    ifft.execute();

    //copy to buffer and discard overlap
    int outputsamples=chan.size - chan.ovlskip;
    d.resize(outputsamples);
    memcpy(d.data(), ifft.get_outbuf()+chan.ovlskip, outputsamples*sizeof(gr_complex));

    //save to chan
    chan.push_data(d);
}

void fftshift(gr_complex *in, gr_complex *out, int sz){
    if(sz%2)
        sz--;

    int N2=sz/2;

    memcpy( out, in+N2, N2*sizeof(gr_complex) );
    memcpy( out+N2, in, N2*sizeof(gr_complex) );
}

void get_channel_data_hist(const gr_complex *hist, const gr_complex *v, channel &chan){
    get_channel_data(hist, chan);
    get_channel_data(v, chan);
}



void cr_window(int blocksize, float passbw, float stopbw, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize){
    if(passbw>=1.0){
        passbw=1.0;
        stopbw=1.0;
    }else if(stopbw>=1.0){
        stopbw=1.0;
    }

    int lowsamps=(int)((1.0-stopbw) * (double)blocksize) / 2;
    int highsamps=(int)(passbw * (double)blocksize);
    int rampsamps=(blocksize - 2*lowsamps - highsamps)/2;

    step=step%relinvovl;
    w.resize(relinvovl);

    std::vector<double> w_d;
    double v=1.0 ? normalize : 1.0/(double)blocksize;
    w_d.clear();
    w_d.resize(blocksize, v);

    for(int i=0;i<lowsamps;i++){
        w_d[i] = 0.0;
        w_d[blocksize -1 -i] = 0.0;
    }

    double phi;
    for(int i=0;i<rampsamps;i++){
        phi=(double)(i+1) / (double)(rampsamps+1) * M_PI - M_PI/2.0;
        w_d[lowsamps+i] = v*sin(phi)/2.0+0.5;
        w_d[blocksize -lowsamps -1 -i] = w_d[lowsamps+i];
    }

    int count=0;
    for(int i=0;i<relinvovl;i++){
        phi=2.0 * M_PI * (double)count / (double)relinvovl;
        w[i].resize(blocksize);
        for(int k=0;k<blocksize;k++)
            w[i][k]=(std::complex<float>) std::polar( w_d[k], phi );
        count=(count+step)%relinvovl;
    }
}

int nextpow2(int k){
    if(k<=0){
        throw std::invalid_argument(std::string("Can't eval nextpow2 from ")+std::to_string(k)+std::string("\n"));
    }
    return (int) std::pow(2,ceil(log2((double) k)));
}

std::string get_time(){
    time_t rawtime;
    struct tm * timeinfo;
    char p[40];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime (p,80,"%Y-%m-%d-%H-%M-%S",timeinfo);

    std::string s(p);

    return s;
}


} /* namespace FDC */
} /* namespace gr */

