set(coco_srcs
  coco.c
  coco_experiment_rayes.cpp
  )

set(coco_incs
  coco.h
  )

add_executable(coco
  ${coco_srcs}
  ${coco_incs})
target_link_libraries(coco es_rayes es_core)

install(TARGETS coco
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

