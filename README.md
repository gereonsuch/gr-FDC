# gr-FDC

* Author:  Gereon Such
* Email:   gereonsuch@gmail.com
* Website: https://gereonsuch-sdr.com

## Description

gr-FDC is a GnuRadio Out of Tree (OOT) module, implementing several Frequency Domain Channelizers for different purposes combining them in a Python Hier block. 

It's main purpose is channelization of multicarrier signals, i.e. demand assigned multiple access systems or carrier detection in wideband signals. 

gr-FDC is unsuitable for Code division multiple access systems. 

## Installation

If you found thism you propably know how to install a GnuRadio OOT Module:

'''bash
$ git clone https://github.com/gereonsuch/gr-FDC.git
$ cd gr-FDC
$ mkdir build
$ cd build
$ cmake .. && make && sudo make install && sudo ldconfig
'''

## Usage

Primarily, you should start by using the block FrequencyDomainChannelizer, as it is a Hier block containing all implemented functionality and adjusts parameters to your needs. The Python way. 

Feel free to use other blocks seperately, but this will most likely not leed to any performance gain whatsoever, since the hier itself block only instanciates what is necessarily needed. 

## Disclaimer

This is free software and no warranty whatsoever is given. Use and modify it responsibly. 
