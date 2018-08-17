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

#ifndef FDC_WINDOWS_H
#define FDC_WINDOWS_H

#include <math.h>
#include <complex>
#include <vector>

enum WINDOWTYPES{
    RECTANGULAR,
    HANN,
    RAMP
};

void cr_win(int wintype, int blocksize, float passbw, float stopbw, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize=false);
void cr_win(int wintype, int blocksize, int lowsamps, int rampsamps, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize=false);

void cr_rect_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize=false);
void cr_hann_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize=false);
void cr_ramp_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize=false);

void cr_win(int wintype, int blocksize, float passbw, float stopbw, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize){
    if(passbw>=1.0){
        passbw=1.0;
        stopbw=1.0;
        wintype=(int)RECTANGULAR;
    }else if(stopbw>=1.0){
        stopbw=1.0;
    }

    int lowsamps=(int)((1.0-stopbw) * (double)blocksize) / 2;
    int highsamps=(int)(passbw * (double)blocksize);
    int rampsamps=(blocksize - 2*lowsamps - highsamps)/2;

    cr_win(wintype, blocksize, lowsamps, rampsamps, w, relinvovl, step, normalize);
}

void cr_win(int wintype, int blocksize, int lowsamps, int rampsamps, std::vector< std::vector< std::complex<float> > > &w, int relinvovl, int step, bool normalize){
    step=step%relinvovl;
    w.resize(relinvovl);

    std::vector<double> w_d;
    if(wintype==(int)HANN)
        cr_hann_win(blocksize, lowsamps, rampsamps, w_d, normalize);
    else if(wintype==(int)RAMP)
        cr_ramp_win(blocksize, lowsamps, rampsamps, w_d, normalize);
    else
        cr_rect_win(blocksize, lowsamps, rampsamps, w_d, normalize);

    double phi;
    int count=0;
    for(int i=0;i<relinvovl;i++){
        phi=2.0 * M_PI * (double)count / (double)relinvovl;
        w[i].resize(blocksize);
        for(int k=0;k<blocksize;k++)
            w[i][k]=(std::complex<float>) std::polar( w_d[k], phi );
        count=(count+step)%relinvovl;
    }
}

void cr_rect_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize){
    double v=normalize? 1.0 : 1.0/(double)blocksize;

    w.clear();
    w.resize(blocksize, v);

    for(int i=0;i<lowsamps+rampsamps/2;i++){
        w[i] = 0.0;
        w[blocksize -1 -i] = 0.0;
    }
}

void cr_ramp_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize){
    double v=normalize? 1.0 : 1.0/(double)blocksize;
    w.clear();
    w.resize(blocksize, v);

    for(int i=0;i<lowsamps;i++){
        w[i] = 0.0;
        w[blocksize -1 -i] = 0.0;
    }

    for(int i=0;i<rampsamps;i++){
        w[lowsamps+i] = v*(double)(i+1) / (double)(rampsamps+1);
        w[blocksize -lowsamps -1 -i] = w[lowsamps+i];
    }
}

void cr_hann_win(int blocksize, int lowsamps, int rampsamps, std::vector<double> &w, bool normalize){
    double v=normalize? 1.0 : 1.0/(double)blocksize;
    w.clear();
    w.resize(blocksize, v);

    for(int i=0;i<lowsamps;i++){
        w[i] = 0.0;
        w[blocksize -1 -i] = 0.0;
    }

    double phi;
    for(int i=0;i<rampsamps;i++){
        phi=(double)(i+1) / (double)(rampsamps+1) * M_PI - M_PI/2.0;
        w[lowsamps+i] = v*sin(phi)/2.0+0.5;
        w[blocksize -lowsamps -1 -i] = w[lowsamps+i];
    }
}


#endif /* FDC_WINDOWS_H */

