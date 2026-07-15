set(mc_state_observation_SRC conversions/kinematics.cpp)

set(mc_state_observation_HDR
    ${CMAKE_SOURCE_DIR}/include/mc_state_observation/conversions/kinematics.h)

add_library(mc_state_observation SHARED ${mc_state_observation_SRC}
                                        ${mc_state_observation_HDR})

target_link_libraries(mc_state_observation PUBLIC mc_rtc::mc_control)
install(
  TARGETS mc_state_observation
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")

option(WITH_KINETICS_OBSERVER_FG "Build MCKineticsObserverFG" OFF)

macro(add_simple_observer observer_name)
  add_observer(${observer_name} "${observer_name}.cpp"
               "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
  # mc_observers doesn't actually depend on mc_control, but the observer's
  # implementation does
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_control)
endmacro()

macro(add_so_observer observer_name)
  add_simple_observer(${observer_name})
  target_link_libraries(
    ${observer_name} PUBLIC state-observation::state-observation
                            mc_state_observation)
endmacro()

macro(add_kineticsobserver)
  add_observer(
    MCKineticsObserver "MCKineticsObserver.cpp"
    "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/MCKineticsObserver.h")
  # mc_observers doesn't actually depend on mc_control, but the observer's
  # implementation does
  target_link_libraries(
    MCKineticsObserver
    PUBLIC mc_rtc::mc_control state-observation::state-observation
           mc_state_observation MCValinor)
endmacro()

macro(add_valinor)
  add_observer(MCValinor "MCValinor.cpp"
               "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/MCValinor.h")
  # mc_observers doesn't actually depend on mc_control, but the observer's
  # implementation does
  target_link_libraries(
    MCValinor PUBLIC mc_rtc::mc_control state-observation::state-observation
                     mc_state_observation TiltObserver)
endmacro()

macro(add_kineticsobserver_fg)
  find_package(kinetics_observer_fg REQUIRED CONFIG)

  add_observer(
    MCKineticsObserverFG "MCKineticsObserverFG.cpp"
    "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/MCKineticsObserverFG.h")

  target_link_libraries(
    MCKineticsObserverFG
    PUBLIC mc_rtc::mc_control
           state-observation::state-observation
           mc_state_observation
           kinetics_observer_fg::kinetics_observer_fg)

  get_target_property(KINETICS_OBSERVER_FG_INCLUDE_DIRS
                      kinetics_observer_fg::kinetics_observer_fg
                      INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(MCKineticsObserverFG BEFORE PRIVATE
                             ${KINETICS_OBSERVER_FG_INCLUDE_DIRS})
endmacro()

macro(add_observer_with_filter observer_name)
  add_observer(
    ${observer_name}
    "${observer_name}.cpp;filtering.cpp"
    "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h;${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/filtering.h"
  )
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_control)
endmacro()

add_so_observer(AttitudeObserver)
# add_so_observer(MCValinor)
add_so_observer(MCWaiko)
add_so_observer(TiltObserver)
# add_so_observer(MCVanytEstimator) add_so_observer(MCViking)
add_so_observer(MocapVisualizer)

add_kineticsobserver()
add_valinor()

if(WITH_KINETICS_OBSERVER_FG)
  list(PREPEND CMAKE_PREFIX_PATH
      "/home/arnaud/devel/src/gtsam/install"
      "/home/arnaud/devel/install"
    )
  add_kineticsobserver_fg()
endif()

if(WITH_ROS_OBSERVERS)
  add_simple_observer(MocapObserver)
  add_simple_observer(MocapObserverROS)
  target_link_libraries(MocapObserverROS PUBLIC MocapObserver)
  target_link_libraries(MocapObserverROS PUBLIC mc_state_observation::ROS
                                                mc_rtc::mc_rtc_ros)
  set_target_properties(
    MocapObserverROS PROPERTIES INSTALL_RPATH
                                ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})

  find_package(gram_savitzky_golay REQUIRED)
  add_observer_with_filter(SLAMObserver)
  target_link_libraries(
    SLAMObserver
    PUBLIC mc_rtc::mc_control mc_state_observation::ROS mc_rtc::mc_rtc_ros
           gram_savitzky_golay::gram_savitzky_golay)
  set_target_properties(
    SLAMObserver PROPERTIES INSTALL_RPATH
                            ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})

  add_simple_observer(ObjectObserver)
  target_link_libraries(
    ObjectObserver PUBLIC mc_rtc::mc_control mc_state_observation::ROS
                          mc_rtc::mc_rtc_ros)
  set_target_properties(
    ObjectObserver PROPERTIES INSTALL_RPATH
                              ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION include)
