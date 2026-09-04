include_directories(${PROJECT_SOURCE_DIR})

add_executable(block2d block2d.cc)
target_link_libraries(block2d
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(block2d.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/block2d/block2d.yaml COPYONLY)
configure_file(block2d_push.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/block2d/block2d_push.yaml COPYONLY)
configure_file(gait.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/block2d/gait.yaml COPYONLY)

add_test(NAME block2d_test COMMAND block2d --test)
