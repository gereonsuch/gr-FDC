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
        
        self.normwidth=2048
        self.block_reduction=int(self.blocklen//self.normwidth) # if blocklen >= normalize
        self.block_interpolation=int(self.normwidth//self.blocklen) # if blocklen < normalize
        
        self.pixels=numpy.zeros((1,self.normwidth*3), dtype=numpy.uint8)
        
        self.colorscheme_cols, self.colorscheme_bins = self.cr_colorscheme(int(colorscheme))
        
        self.min_redraw_time=0.05
        self.last_drawn=0.0
        self.curdrawing=False
        
        QtGui.QWidget.__init__(self)
        
        self.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding,QtGui.QSizePolicy.MinimumExpanding))
        
        gr.sync_block.__init__(self,
            name="WaterfallMsgTagging",
            in_sig=[(numpy.float32, self.blocklen)],
            out_sig=None)
        
        self.puffer_blocks=[]
        
        #init timer to asynchronously redraw
        self.timer=QtCore.QTimer()
        self.timer.timeout.connect(self.timerSlot)
        self.timer.start(200)
        
    
    def timerSlot(self):
        if self.curdrawing or time.time()-self.last_drawn<self.min_redraw_time:
            return
        
        self.update()
        
    def renew_pixmap(self):
        pixels=numpy.zeros((self.normheight, self.normwidth*3), dtype=numpy.uint8)
        
        minheight=min(self.normheight, self.pixels.shape[0])
        
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
        #decimate consecutive blocks and delete these from puffer
        blocks=numpy.mean( numpy.array(self.puffer_blocks[:numblocks]).reshape(numblocks//self.blockdecimation, self.blockdecimation, self.normwidth), 1)
        del self.puffer_blocks[:numblocks]
        
        lines=blocks.shape[0]
        
        #append lines to pixel array
        self.pixels=numpy.append(self.pixels[lines:],self.apply_colorscheme(blocks) )
        self.pixels=self.pixels.reshape(self.pixels.size//(self.normwidth*3), self.normwidth*3)
        
        


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
        
        print('colorscheme: {}\n\n'.format(colorscheme))
        
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
        elif colorscheme==2:
            Np=int(N/2)
            colorscheme_cols=numpy.array([numpy.concatenate(( lsp(0,255,Np),[255]*Np )),
                                          numpy.concatenate(( [0]*Np,lsp(0,255,Np) )),
                                          [0]*N ], dtype=numpy.uint8).transpose().reshape(N, 3)
        elif colorscheme==3:
            colorscheme_cols=numpy.kron(numpy.linspace(0,255,N, dtype=numpy.uint8), [1,1,1]).reshape(N, 3)
        else:
            colorscheme_cols=numpy.array([[0]*N,
                                          numpy.concatenate(([0]*int(N/2), lsp(0,255,int(N/2)) )),
                                          numpy.concatenate((lsp(0,255,int(N/2)), [255]*int(N/2) )) ], dtype=numpy.uint8).transpose().reshape(N, 3)
        
        return colorscheme_cols, colorscheme_bins
    
    
    
