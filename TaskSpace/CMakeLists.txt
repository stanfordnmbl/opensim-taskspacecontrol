
set(FILES
    osimTaskSpaceControlDLL.h
    RegisterTypes_osimTaskSpaceControl.cpp
    RegisterTypes_osimTaskSpaceControl.h
    Controller.cpp
    Controller.h
    PriorityLevel.cpp
    PriorityLevel.h
    PriorityLevelSet.h
    Task.cpp
    Task.h
    TaskSet.h
    StationTask.cpp
    StationTask.h
    StationTrackingTask.cpp
    StationTrackingTask.h
    ConstantStationTrackingTask.cpp
    ConstantStationTrackingTask.h
    GravityCompensationTask.h
    )
add_library(osimTaskSpaceControl SHARED ${FILES})
target_link_libraries(osimTaskSpaceControl ${OPENSIMSIMBODY_LIBRARIES})

# For Windows, to export symbols. See osimTaskSpaceControlDLL.h
set_target_properties(osimTaskSpaceControl PROPERTIES
    DEFINE_SYMBOL OSIM${UKIT}_EXPORTS)
