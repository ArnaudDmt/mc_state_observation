option(BUILD_MCKINETICS_ONLY "" OFF)

macro(add_simple_observer observer_name)
  add_observer(${observer_name} "${observer_name}.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
  # mc_observers doesn't actually depend on mc_control, but the observer's implementation does
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_control)
  # Force installation in non-debug folder
  if("${CMAKE_BUILD_TYPE}" STREQUAL Debug)
    install(TARGETS ${observer_name} DESTINATION ${MC_RTC_LIBDIR}/mc_observers)
  endif()
endmacro()

macro(add_so_observer observer_name)
  add_simple_observer(${observer_name})
  target_link_libraries(${observer_name} PUBLIC state-observation::state-observation)
endmacro()

macro(add_observer_with_filter observer_name)
  add_observer(${observer_name} "${observer_name}.cpp;filtering.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h;${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/filtering.h")
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_control)
endmacro()

if(NOT BUILD_MCKINETICS_ONLY)
  add_so_observer(AttitudeObserver)
  add_so_observer(LegacyFlexibilityObserver)
endif()
add_so_observer(MCKineticsObserver)

if(WITH_ROS_OBSERVERS AND NOT BUILD_MCKINETICS_ONLY)
  add_simple_observer(MocapObserver)
  add_simple_observer(MocapObserverROS)
  target_link_libraries(MocapObserverROS PUBLIC MocapObserver)
  target_link_libraries(MocapObserverROS PUBLIC mc_state_observation::ROS mc_rtc::mc_rtc_ros)
  set_target_properties(MocapObserverROS PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})

  find_package(gram_savitzky_golay REQUIRED)
  add_observer_with_filter(SLAMObserver)
  target_link_libraries(SLAMObserver PUBLIC mc_rtc::mc_control mc_state_observation::ROS mc_rtc::mc_rtc_ros gram_savitzky_golay::gram_savitzky_golay)
  set_target_properties(SLAMObserver PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})

  add_simple_observer(ObjectObserver)
  target_link_libraries(ObjectObserver PUBLIC mc_rtc::mc_control mc_state_observation::ROS mc_rtc::mc_rtc_ros)
  set_target_properties(ObjectObserver PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
