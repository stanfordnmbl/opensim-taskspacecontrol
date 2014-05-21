
include_directories(${CMAKE_SOURCE_DIR})

add_executable(testFourLinks testFourLinks.cpp)

target_link_libraries(testFourLinks
    ${OPENSIMSIMBODY_LIBRARIES}
    osimTaskSpaceControl)

add_executable(testHumanPosture testHumanPosture.cpp)

target_link_libraries(testHumanPosture
    ${OPENSIMSIMBODY_LIBRARIES}
    osimTaskSpaceControl)
