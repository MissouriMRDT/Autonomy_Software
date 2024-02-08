# Find library path and object files, store them in INCLUDE_DIR and LIBS variables.
set(TENSORFLOWLITE_INCLUDE_DIRS "/usr/local/include/tensorflow")
find_library(TENSORFLOWLITE_LIBS NAMES libtensorflow-lite.a PATHS /usr/local/lib)

# Include package handler and create new package.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TENSORFLOWLITE DEFAULT_MSG TENSORFLOWLITE_LIBS TENSORFLOWLITE_INCLUDE_DIRS)

mark_as_advanced(TENSORFLOWLITE_INCLUDE_DIRS TENSORFLOWLITE_LIBS)
