add_library(fans INTERFACE)

target_sources(fans INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/fans.c
)

target_include_directories(fans INTERFACE ${CMAKE_CURRENT_LIST_DIR})
