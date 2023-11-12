# Find library path and object files, store them in INCLUDE_DIR and LIBS variables.
set(LIBEDGETPU_INCLUDE_DIRS "/usr/local/include/edgetpu")
find_library(LIBEDGETPU_LIBS NAMES libedgetpu.so.1 PATHS /usr/local/lib)

# Include package handler and create new package.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBEDGETPU DEFAULT_MSG LIBEDGETPU_LIBS LIBEDGETPU_INCLUDE_DIRS)

mark_as_advanced(LIBEDGETPU_INCLUDE_DIRS LIBEDGETPU_LIBS)
