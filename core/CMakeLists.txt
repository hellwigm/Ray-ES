set(es_core_srcs
  src/util.cpp
  )

set(es_core_incs
  include/es/core/util.h
  include/es/core/version.h
  )

add_library(es_core ${es_core_srcs} ${es_core_incs})
target_include_directories(es_core PUBLIC include/)
target_include_directories(es_core SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

install(TARGETS es_core
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

install(
    DIRECTORY include/
    DESTINATION include
    )

