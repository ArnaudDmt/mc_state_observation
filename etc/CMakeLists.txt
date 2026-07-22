option(INSTALL_OBSERVER_CONFIG_FILES
       "Install observer configuration files to local directory" ON)

if(${INSTALL_OBSERVER_CONFIG_FILES})
  set(OBSERVERS MCKineticsObserver Tilt MCWaiko MCValinor)

  foreach(OBS ${OBSERVERS})

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${OBS}/robots")
      install(DIRECTORY ${OBS}/robots/
              DESTINATION ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}/${OBS})
    endif()

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${OBS}/${OBS}.yaml")
      install(FILES "${OBS}/${OBS}.yaml"
              DESTINATION ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}/etc)
    endif()

  endforeach()
endif()
