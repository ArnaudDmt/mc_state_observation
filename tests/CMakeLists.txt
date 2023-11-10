add_library(Test_Runner OBJECT test_run.cpp)
target_link_libraries(Test_Runner PUBLIC mc_rtc::mc_control)

function(TestObserver OBSERVER_NAME NR_ITER)
  set(PIPELINE_CFG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Test_${OBSERVER_NAME}.yaml)
  if(NOT EXISTS ${PIPELINE_CFG_FILE})
    message(
      FATAL_ERROR "Missing pipeline configuration for testing ${OBSERVER_NAME}")
  endif()
  file(READ ${PIPELINE_CFG_FILE} OBSERVER_CFG)
  set(CFG_CONFIGURED_OUT
      "${CMAKE_CURRENT_BINARY_DIR}/${OBSERVER_NAME}/mc_rtc.in.yaml")
  set(CFG_OUT
      "${CMAKE_CURRENT_BINARY_DIR}/${OBSERVER_NAME}/$<CONFIG>/etc/mc_rtc.yaml")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.in.yaml"
                 "${CFG_CONFIGURED_OUT}" @ONLY)
  file(
    GENERATE
    OUTPUT ${CFG_OUT}
    INPUT ${CFG_CONFIGURED_OUT})
  set(CPP_CONFIGURED_OUT
      "${CMAKE_CURRENT_BINARY_DIR}/${OBSERVER_NAME}/config.in.cpp")
  set(CPP_OUT
      "${CMAKE_CURRENT_BINARY_DIR}/${OBSERVER_NAME}/$<CONFIG>/config.cpp")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.in.cpp
                 ${CPP_CONFIGURED_OUT} @ONLY)
  file(
    GENERATE
    OUTPUT ${CPP_OUT}
    INPUT ${CPP_CONFIGURED_OUT})
  add_executable(Test_${OBSERVER_NAME} ${CPP_OUT})
  target_link_libraries(Test_${OBSERVER_NAME} PUBLIC Test_Runner)
  add_test(NAME Test_${OBSERVER_NAME} COMMAND Test_${OBSERVER_NAME})
endfunction()

testobserver(Attitude 100)
testobserver(MCKineticsObserver 100)
testobserver(NaiveOdometry 100)
# FIXME Cannot pass until MOCAPVisualizer runs correctly without a file
# TestObserver(MOCAPVisualizer 0)
testobserver(Tilt 100)

if(WITH_ROS_OBSERVERS)
  testobserver(MocapObserverROS 100)
  testobserver(SLAM 100)
  testobserver(Object 100)
endif()
