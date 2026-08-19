include_directories(${PROJECT_SOURCE_DIR})

add_executable(block1d block1d.cc)
target_link_libraries(block1d
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(block1d.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/block1d/block1d.yaml COPYONLY)

add_test(NAME block1d_test COMMAND block1d --test)
