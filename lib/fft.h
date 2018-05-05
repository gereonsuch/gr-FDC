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

#ifndef FDC_FFT_H
#define FDC_FFT_H

#include <math.h>
#include <complex>
#include <vector>
#include <stdio.h>
#include <string.h>

#define gr_complex std::complex<float>

template <typename T>
T get_MSB(T N){
    int k=0;

    if(N==(T)0)
        return 0;

    while( (N>>k) != (T)1)
        k++;

    return k+1;
}

template <typename T>
T get_next_pow2(T N){
    int k=0;

    if(N==0)
        return 0;

    while( (N>>k) != (T)1)
        k++;

    if(N==((T)1)<<k)
        return N;

    return ((T)1)<<(k+1);
}

int bitinverse(int k, int N){
    int v=0;
    for(int i=0;i<N;i++)
        if(k&(1<<i))
            v^=1<<(N-1-i);
    return v;
}

template <typename T>
void fft(std::vector<T> &in, std::vector<T> &out){
    std::vector<T> tmp=in;
    tmp.resize( get_next_pow2(tmp.size()), (T) 0.0 );
    out.resize(tmp.size());

    int used_bits=get_MSB(tmp.size())-1;

    for(int i=0;i<tmp.size();i++)
        out[ i ] = tmp[ bitinverse(i, used_bits) ];

    //in is left untouched since fft might be used for measuring purpose only
    std::vector<T> exptable;
    exptable.resize(out.size());
    double factor=-2.0 * M_PI / (double) exptable.size();
    for(int i=0;i<exptable.size();i++)
        exptable[i] = (T)std::polar(1.0, factor * (double)i);

    int numblocks, blockhalf;
    T a,b;

    for(int blocksize=2;blocksize<=out.size();blocksize<<=1){
        numblocks=out.size()/blocksize;
        blockhalf=blocksize/2;
        for(int block=0;block<numblocks;block++){
            for(int i=0;i<blockhalf;i++){
                a=out[block*blocksize + i];
                b=out[block*blocksize + blockhalf+i];

                out[block*blocksize + i] = a + b*exptable[i * numblocks];
                out[block*blocksize + blockhalf+i] = a - b*exptable[i * numblocks];

                //fft without lookup table
                //out[block*blocksize + i] = a + b*(T)std::polar(1.0, -2.0*M_PI/(double)blocksize*(double)i);
                //out[block*blocksize + blockhalf+i] = a - b*(T)std::polar(1.0, -2.0*M_PI/(double)blocksize*(double)i);
            }
        }
    }
}

template <typename T>
void ifft(std::vector<T> &in, std::vector<T> &out){
    std::vector<T> tmp=in;
    tmp.resize( get_next_pow2(tmp.size()), (T) 0.0 );
    out.resize(tmp.size());

    int used_bits=get_MSB(tmp.size())-1;

    for(int i=0;i<tmp.size();i++)
        out[ i ] = tmp[ bitinverse(i, used_bits) ];

    //in is left untouched since fft might be used for measuring purpose only
    std::vector<T> exptable;
    exptable.resize(out.size());
    double factor=2.0 * M_PI / (double) exptable.size();
    for(int i=0;i<exptable.size();i++)
        exptable[i] = (T)std::polar(1.0, factor * (double)i);

    int numblocks, blockhalf;
    T a,b;

    for(int blocksize=2;blocksize<=out.size();blocksize<<=1){
        numblocks=out.size()/blocksize;
        blockhalf=blocksize/2;
        for(int block=0;block<numblocks;block++){
            for(int i=0;i<blockhalf;i++){
                a=out[block*blocksize + i];
                b=out[block*blocksize + blockhalf+i];

                out[block*blocksize + i] = a + b*exptable[i * numblocks];
                out[block*blocksize + blockhalf+i] = a - b*exptable[i * numblocks];

                //fft without lookup table
                //out[block*blocksize + i] = a + b*(T)std::polar(1.0, -2.0*M_PI/(double)blocksize*(double)i);
                //out[block*blocksize + blockhalf+i] = a - b*(T)std::polar(1.0, -2.0*M_PI/(double)blocksize*(double)i);
            }
        }
    }
}

template <typename T>
void fftshift(std::vector<T> &in, std::vector<T> &out){
    if(in.size()%2)
        in.resize(in.size()+1, (T) 0.0);

    out.resize(in.size());
    memcpy( out.data(),  in.data()+in.size()/2, in.size()/2*sizeof(T) );
    memcpy( out.data()+out.size()/2, in.data(), out.size()/2*sizeof(T) );
}


#endif /* FDC_FFT_H */

