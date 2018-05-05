# Copyright 2011,2012,2016 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR}
    /usr/lib/x86_64-linux-gnu)
link_directories(${Boost_LIBRARY_DIRS}
    /usr/lib/x86_64-linux-gnu)


find_package(Threads REQUIRED)

list(APPEND FDC_sources
    overlap_save_impl.cc
    phase_shifting_windowing_vcc_impl.cc
    vector_cut_vxx_impl.cc
    activity_controlled_channelizer_vcm_impl.cc
    activity_detection_channelizer_vcm_impl.cc
)

set(FDC_sources "${FDC_sources}" PARENT_SCOPE)
if(NOT FDC_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT FDC_sources)

add_library(gnuradio-FDC SHARED ${FDC_sources})

target_link_libraries(gnuradio-FDC ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(gnuradio-FDC PROPERTIES DEFINE_SYMBOL "gnuradio_FDC_EXPORTS")


if(APPLE)
    set_target_properties(gnuradio-FDC PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-FDC RUNTIME_COMPONENT "FDC_runtime" DEVEL_COMPONENT "FDC_devel")

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(${CPPUNIT_INCLUDE_DIRS})

list(APPEND test_FDC_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/test_FDC.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/qa_FDC.cc
)

add_executable(test-FDC ${test_FDC_sources})

target_link_libraries(
  test-FDC
  ${GNURADIO_RUNTIME_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CPPUNIT_LIBRARIES}
  gnuradio-FDC
)

GR_ADD_TEST(test_FDC test-FDC)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

