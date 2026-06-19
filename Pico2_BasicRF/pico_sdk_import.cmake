# Keep the project self-contained while using the Pico SDK selected by
# PICO_SDK_PATH. The included file is the SDK's official import helper.
if(NOT DEFINED ENV{PICO_SDK_PATH})
    message(FATAL_ERROR "PICO_SDK_PATH is not set")
endif()

include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
