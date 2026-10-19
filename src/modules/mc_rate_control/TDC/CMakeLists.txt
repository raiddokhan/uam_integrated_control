
px4_add_library(TDC
	TDC.cpp
    TDC.hpp
)
target_compile_options(TDC PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(TDC PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

