find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_FDC gnuradio-FDC)

FIND_PATH(
    GR_FDC_INCLUDE_DIRS
    NAMES gnuradio/FDC/api.h
    HINTS $ENV{FDC_DIR}/include
        ${PC_FDC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_FDC_LIBRARIES
    NAMES gnuradio-FDC
    HINTS $ENV{FDC_DIR}/lib
        ${PC_FDC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-FDCTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_FDC DEFAULT_MSG GR_FDC_LIBRARIES GR_FDC_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_FDC_LIBRARIES GR_FDC_INCLUDE_DIRS)
