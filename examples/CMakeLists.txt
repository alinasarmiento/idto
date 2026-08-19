include_directories(${PROJECT_SOURCE_DIR})

add_library(yaml_config INTERFACE yaml_config.h)

add_library(pd_plus_controller
            pd_plus_controller.cc
            pd_plus_controller.h)
target_link_libraries(pd_plus_controller drake::drake)

add_library(mpc_controller
            mpc_controller.cc
            mpc_controller.h)
target_link_libraries(mpc_controller 
                      drake::drake
                      pd_plus_controller
                      trajectory_optimizer)

add_library(example_base example_base.cc example_base.h)
target_link_libraries(example_base
                      drake::drake
                      mpc_controller
                      yaml_config
                      find_resource
		      dairlibtypes
		      standalone_lcm_systems
		      lcm_driven_loop)

add_library(standalone_lcm_systems
            standalone_lcm_systems.cc
            standalone_lcm_systems.h)
target_link_libraries(standalone_lcm_systems 
		      drake::drake
		      dairlibtypes)
		    
add_library(lcm_driven_loop INTERFACE)
target_include_directories(lcm_driven_loop
                           INTERFACE
			   ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lcm_driven_loop
                      INTERFACE
		      drake::drake
		      dairlibtypes)

add_subdirectory(acrobot)
add_subdirectory(airhockey)
add_subdirectory(allegro_hand)
add_subdirectory(dual_jaco)
add_subdirectory(hopper)
add_subdirectory(jaco)
add_subdirectory(jaco_ball)
add_subdirectory(kuka)
add_subdirectory(mini_cheetah)
add_subdirectory(pendulum)
add_subdirectory(punyo)
add_subdirectory(spinner)
add_subdirectory(block1d)
add_subdirectory(block2d)
