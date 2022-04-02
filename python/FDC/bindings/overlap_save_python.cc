/*
 * Copyright 2022 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually edited  */
/* The following lines can be configured to regenerate this file during cmake      */
/* If manual edits are made, the following tags should be modified accordingly.    */
/* BINDTOOL_GEN_AUTOMATIC(0)                                                       */
/* BINDTOOL_USE_PYGCCXML(0)                                                        */
/* BINDTOOL_HEADER_FILE(overlap_save.h)                                        */
/* BINDTOOL_HEADER_FILE_HASH(3fa35833b24f16b8d633ce9a3653c0b4)                     */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <gnuradio/FDC/overlap_save.h>
// pydoc.h is automatically generated in the build directory
#include <overlap_save_pydoc.h>

void bind_overlap_save(py::module& m)
{

    using overlap_save    = ::gr::FDC::overlap_save;


    py::class_<overlap_save, gr::sync_block, gr::block, gr::basic_block,
        std::shared_ptr<overlap_save>>(m, "overlap_save", D(overlap_save))

        .def(py::init(&overlap_save::make),
           py::arg("itemsize"),
           py::arg("outputlen"),
           py::arg("overlaplen"),
           D(overlap_save,make)
        )
        



        ;




}








