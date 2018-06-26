#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2018 Gereon Such.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy
from gnuradio import gr
from PyQt4 import Qt, QtCore, QtGui
import pmt
import time

class WaterfallMsgTagging(gr.sync_block, QtGui.QWidget):
    """
    docstring for block WaterfallMsgTagging
    """
    def __init__(self, blocklen, samp_rate, relinvovl, blockdecimation, loginput, minvaldb, maxvaldb, colorscheme, tagmode):
        self.blocklen=int(blocklen)
        self.samp_rate=float(samp_rate)
        self.relinvovl=int(relinvovl)
        self.data_rate=self.samp_rate * (1.0-1.0/float(relinvovl))
        self.blockdecimation=int(blockdecimation) if int(blockdecimation)>0 else 1
        
        self.loginput=bool(loginput)
        self.minvaldb=float(minvaldb)
        self.maxvaldb=float(maxvaldb)
        
        self.normwidth=1024
        self.block_reduction=int(self.blocklen//self.normwidth) # if blocklen >= normalize
        self.block_interpolation=int(self.normwidth//self.blocklen) # if blocklen < normalize
        
        self.pixels=numpy.zeros((1,self.normwidth*3), dtype=numpy.uint8)
        
        self.colorscheme_cols, self.colorscheme_bins, self.colorscheme_frame = self.cr_colorscheme(int(colorscheme))
        
        self.min_redraw_time=0.05
        self.last_drawn=0.0
        self.curdrawing=False
        
        QtGui.QWidget.__init__(self)
        
        self.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding,QtGui.QSizePolicy.MinimumExpanding))
        
        gr.sync_block.__init__(self,
            name="WaterfallMsgTagging",
            in_sig=[(numpy.float32, self.blocklen)],
            out_sig=None)
        
        self.min_block=-self.pixels.shape[0]
        self.max_block=0
        self.puffer_blocks=[]
        self.msg_puffer=[]
        
        #init timer to asynchronously redraw
        self.timer=QtCore.QTimer()
        self.timer.timeout.connect(self.timerSlot)
        self.timer.start(200)
        
        self.msgport=pmt.intern('msgin')
        self.message_port_register_in(self.msgport)
        self.set_msg_handler(self.msgport, self.msg_handler)
        
    
    def timerSlot(self):
        if self.curdrawing or time.time()-self.last_drawn<self.min_redraw_time:
            return
        
        self.update()
        
    def msg_handler(self, m):
        if not pmt.is_pair(m):
            return
        meta=pmt.car(m)
        
        if not pmt.is_dict(meta):
            return
        
        blockstart=pmt.to_long(pmt.dict_ref(meta, pmt.intern('blockstart'), pmt.from_long(-1024)))
        blockend=pmt.to_long(pmt.dict_ref(meta, pmt.intern('blockend'), pmt.from_long(-1024)))
        
        if blockstart==-1024 or blockend==-1024:
            return
        
        rel_cfreq=pmt.to_double(pmt.dict_ref(meta, pmt.intern('rel_cfreq'), pmt.from_double(-1.0)))
        rel_bw=pmt.to_double(pmt.dict_ref(meta, pmt.intern('rel_bw'), pmt.from_double(-1.0)))
        
        if rel_cfreq<0.0 or rel_bw<0.0:
            return
        
        blockleft=int(self.normwidth*(rel_cfreq-rel_bw/2.0))
        blockright=int(numpy.ceil(self.normwidth*(rel_cfreq+rel_bw/2.0)))
        
        #print('new msg: {} {} {} {}   {} {}'.format(blockstart, blockend, rel_cfreq, rel_bw, blockleft, blockright))
        
        self.msg_puffer+=[(blockstart, blockend, blockleft, blockright)]
        
    def renew_pixmap(self):
        pixels=numpy.zeros((self.normheight, self.normwidth*3), dtype=numpy.uint8)
        
        minheight=min(self.normheight, self.pixels.shape[0])
        
        #update minimum block counter
        self.min_block+=(self.pixels.shape[0]-self.normheight) * self.blockdecimation
        
        #draw old pixels to bottom of new. 
        pixels[self.normheight-minheight:]=self.pixels[self.pixels.shape[0]-minheight:]
        
        #replace old with new pixmap
        self.pixels=pixels
    
    def resizeEvent(self, e=None):
        self.normheight=int(round(float(self.height()) *float(self.normwidth)/float(self.width())))
        self.renew_pixmap()
        QtGui.QWidget.resizeEvent(self, e)
        
    
    def paintEvent(self, e=None):
        self.curdrawing=True
        if time.time()-self.last_drawn < self.min_redraw_time:
            return
        
        self.pxupdate()
        
        QtGui.QWidget.paintEvent(self, e)
        
        #draw Pixmap to Widget
        p=QtGui.QPainter(self)
        
        qimg=QtGui.QImage(self.pixels, self.normwidth, self.normheight, QtGui.QImage.Format_RGB888)
        pxmap=QtGui.QPixmap()
        pxmap.convertFromImage(qimg)
        
        p.drawPixmap(self.rect(), pxmap)
        
        self.last_drawn=time.time()
        self.curdrawing=False
        
    def pxupdate(self):
        if len(self.puffer_blocks)<self.blockdecimation:
            return
        numblocks=len(self.puffer_blocks)-(len(self.puffer_blocks)%self.blockdecimation)

        #increase block counters        
        self.min_block+=numblocks
        self.max_block+=numblocks
        
        #decimate consecutive blocks and delete these from puffer
        blocks=numpy.mean( numpy.array(self.puffer_blocks[:numblocks]).reshape(numblocks//self.blockdecimation, self.blockdecimation, self.normwidth), 1)
        del self.puffer_blocks[:numblocks]
        
        lines=blocks.shape[0]
        
        #append lines to pixel array
        self.pixels=numpy.append(self.pixels[lines:],self.apply_colorscheme(blocks) )
        self.pixels=self.pixels.reshape(self.pixels.size//(self.normwidth*3), self.normwidth*3)
        
        #eval messages
        for i in list(reversed(range(len(self.msg_puffer)))):
            blockstart, blockend, blockleft, blockright = self.msg_puffer[i]
            if blockend<=self.min_block: #signal too old
                del self.msg_puffer[i]
                continue
            elif blockstart>=self.max_block: #signal fully ahead
                continue
            else:
                if blockend<self.max_block and blockstart>self.min_block: #signal fully in window
                    self.draw_rect(blockstart, blockend, blockleft, blockright)
                    del self.msg_puffer[i]
                    continue
                elif blockstart<=self.min_block: #only signal begin out of range
                    self.draw_h_line(blockend, blockleft, blockright)
                    self.draw_v_line(blockend, blockleft, blockright)
                    del self.msg_puffer[i]
                    continue
                elif blockend>=self.max_block:
                    self.draw_h_line(blockstart, blockleft, blockright)
                    self.draw_v_line(blockend, blockleft, blockright, False)
                    continue #dont delete, block end might appear soon...
                    
                
        
    def draw_rect(self,blockstart, blockend, blockleft, blockright):
        begin=self.normheight - int(numpy.ceil(float(self.max_block-blockstart)/self.blockdecimation))
        end=self.normheight - int(float(self.max_block-blockend)/self.blockdecimation)
        if end==self.normheight:
            end-=1
        
        #print('rect {} {} {} {}   {} {}'.format(blockstart, blockend, blockleft, blockright, begin,end))
        
        hline=numpy.kron(numpy.ones(blockright-blockleft, dtype=numpy.uint8), self.colorscheme_frame)
        vline=numpy.kron(numpy.ones((end-begin,1), dtype=numpy.uint8), self.colorscheme_frame)
        
        self.pixels[begin:end, blockleft*3:blockleft*3+3]=vline
        self.pixels[begin:end, blockright*3:blockright*3+3]=vline
        self.pixels[begin, blockleft*3:blockright*3]=hline
        self.pixels[end, blockleft*3:blockright*3]=hline
    
    def draw_h_line(self, blocknum, blockleft, blockright):
        linenum=self.normheight - max(int(float(self.max_block-blocknum)/self.blockdecimation), 1)
        
        #print('hline {} {} {}     {}'.format(blocknum, blockleft, blockright, linenum))
        
        hline=numpy.kron(numpy.ones(blockright-blockleft, dtype=numpy.uint8), self.colorscheme_frame)
        self.pixels[linenum, blockleft*3:blockright*3]=hline
    
    def draw_v_line(self, blocknum, blockleft, blockright, up=True, length=4):
        linenum=self.normheight - int(float(self.max_block-blocknum)/self.blockdecimation)
        
        #print('vline {} {} {}     {}'.format(blocknum, blockleft, blockright, linenum))
        
        if up:
            if linenum<length:
                length=linenum
                if length<=0:
                    return
            vline=numpy.kron(numpy.ones((length,1), dtype=numpy.uint8), self.colorscheme_frame)
            self.pixels[linenum-length:linenum, blockleft*3:blockleft*3+3]=vline
            self.pixels[linenum-length:linenum, blockright*3:blockright*3+3]=vline
        else:
            if self.pixels.shape[0]-linenum<length:
                length=self.pixels.shape[0]-linenum
                if length<=0:
                    return
            vline=numpy.kron(numpy.ones((length,1), dtype=numpy.uint8), self.colorscheme_frame)
            self.pixels[linenum:linenum+length, blockleft*3:blockleft*3+3]=vline
            self.pixels[linenum:linenum+length, blockright*3:blockright*3+3]=vline
        
        
        


    def work(self, input_items, output_items):
        in0 = input_items[0]
        
        # append data with normalized size to intern puffer
        if self.blocklen>self.normwidth:
            self.puffer_blocks += list(numpy.mean( in0.reshape( in0.shape[0], self.normwidth, self.block_reduction), 2 ))
        else:
            self.puffer_blocks += list(numpy.kron(in0, [1]*self.block_interpolation))
        
        return len(input_items[0])
    
    def apply_colorscheme(self, blocks):
        return self.colorscheme_cols[numpy.digitize(blocks, self.colorscheme_bins, False)].astype(numpy.uint8).reshape(blocks.size*3)
    
    def cr_colorscheme(self, colorscheme):
        # 0 Black-Blue-Cyan-White
        # 1 Black-Rainbow
        # 2 Black-Red-Yellow
        # 3 Black-White
        
        #print('colorscheme: {}\n\n'.format(colorscheme))
        
        N=1024
        colorscheme_bins=numpy.linspace(self.minvaldb, self.maxvaldb, N-1) #N-1 to apply values bigger maxvaldb
        if not self.loginput:
            colorscheme_bins=10.0 ** (colorscheme_bins/10.0)
        
        lsp=lambda *x: numpy.linspace(*x, dtype=numpy.uint8)
        
        if colorscheme==1:
            Np=int(N/4)
            colorscheme_cols=numpy.array([numpy.concatenate(( lsp(0,75,Np),lsp(75,0,Np),[0]*Np,lsp(0,255,Np)  )),
                                          numpy.concatenate(( [0]*Np,[0]*Np,lsp(0,255,Np),[255]*Np )),
                                          numpy.concatenate(( lsp(0,130,Np),lsp(130,255,Np),lsp(255,0,Np),[0]*Np )) ], dtype=numpy.uint8).transpose().reshape(N, 3)
            colorscheme_frame=numpy.array([255,255,255], dtype=numpy.uint8)
        elif colorscheme==2:
            Np=int(N/2)
            colorscheme_cols=numpy.array([numpy.concatenate(( lsp(0,255,Np),[255]*Np )),
                                          numpy.concatenate(( [0]*Np,lsp(0,255,Np) )),
                                          [0]*N ], dtype=numpy.uint8).transpose().reshape(N, 3)
            colorscheme_frame=numpy.array([255,255,255], dtype=numpy.uint8)
        elif colorscheme==3:
            colorscheme_cols=numpy.kron(numpy.linspace(0,255,N, dtype=numpy.uint8), [1,1,1]).reshape(N, 3)
            colorscheme_frame=numpy.array([0,255,0], dtype=numpy.uint8)
        else:
            colorscheme_cols=numpy.array([[0]*N,
                                          numpy.concatenate(([0]*int(N/2), lsp(0,255,int(N/2)) )),
                                          numpy.concatenate((lsp(0,255,int(N/2)), [255]*int(N/2) )) ], dtype=numpy.uint8).transpose().reshape(N, 3)
            colorscheme_frame=numpy.array([255,255,255], dtype=numpy.uint8)
        
        return colorscheme_cols, colorscheme_bins, colorscheme_frame
    
    
    
