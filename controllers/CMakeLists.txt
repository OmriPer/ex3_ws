set(SRC
  controller_bug1.hpp
  controller_bug1.cpp
  controller_bug2.hpp
  controller_bug2.cpp
  # Add other controller source files here
)

add_library(pipuck_controllers MODULE ${SRC})

target_link_libraries(pipuck_controllers
  PUBLIC
    argos3core_${ARGOS_BUILD_FOR}
    argos3plugin_${ARGOS_BUILD_FOR}_pipuck

)

target_include_directories(pipuck_controllers
  PUBLIC
    ${ARGOS_INCLUDE_DIRS}
)