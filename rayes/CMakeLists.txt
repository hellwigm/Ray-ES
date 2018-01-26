set(es_rayes_srcs
  src/RayEs.cpp
  src/Info.cpp
  src/Individual.cpp
  )

set(es_rayes_incs
  include/es/rayes/RayEs.h
  include/es/rayes/Info.h
  )

add_library(es_rayes
  ${es_rayes_srcs}
  ${es_rayes_incs})
target_link_libraries(es_rayes es_core)
target_include_directories(es_rayes PUBLIC include/)
target_include_directories(es_rayes
  SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

add_executable(es_rayestool src/main.cpp)
target_link_libraries(es_rayestool es_rayes)

install(TARGETS es_rayes es_rayestool
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

install(
    DIRECTORY include/
    DESTINATION include
    )
