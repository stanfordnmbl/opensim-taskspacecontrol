
set(FILES
    osimTaskSpaceControlDLL.h
    RegisterTypes_osimTaskSpaceControl.cpp
    RegisterTypes_osimTaskSpaceControl.h
    Controller.cpp
    Controller.h
    )
add_library(osimTaskSpaceControl SHARED ${FILES})
target_link_libraries(osimTaskSpaceControl ${OPENSIMSIMBODY_LIBRARIES})
