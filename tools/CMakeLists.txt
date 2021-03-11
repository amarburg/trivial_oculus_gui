fips_begin_app( oc_client_gui cmdline )
  fips_files(
    oculus_client_gui.cpp
  )

  fips_deps( oculus draw_sonar )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()
