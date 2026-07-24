file(GLOB LCM_TYPE_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.lcm"
)

lcm_wrap_types(
  CPP_HEADERS generated_lcm_headers
  ${LCM_TYPE_FILES}
)

lcm_add_library(dairlibtypes CPP
  ${generated_lcm_headers}
)

target_include_directories(dairlibtypes INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)
