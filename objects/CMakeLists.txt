set(SHARE_DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mc_rtc/${PROJECT_NAME}")

configure_file(alias.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/alias.yaml" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias.yaml" DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/chair_dance.urdf" DESTINATION "${SHARE_DESTINATION}/urdf")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/rsdf/chair_dance/chair_dance.rsdf" DESTINATION "${SHARE_DESTINATION}/rsdf/chair_dance")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/urdf/table_dance.urdf" DESTINATION "${SHARE_DESTINATION}/urdf")
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/rsdf/table_dance/table_dance.rsdf" DESTINATION "${SHARE_DESTINATION}/rsdf/table_dance")
