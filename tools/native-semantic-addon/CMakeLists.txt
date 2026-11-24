# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Production NavigationBasic and NavNBTEvoController semantic guidance addons (ARM QNX).
link_libraries(${aa_runtime_interfaces})

add_library(native_semantic_basic SHARED
  basic_addon.c
  fixture-provider.c
  semantic_cue_map.c
  semantic_handoff.c
  semantic_minimap_render.c
  semantic_minimap_shm.c
  semantic_nds_map.c
  semantic_nds_probe.c
  semantic_xar_patch.c
  semantic_xar_hook.c
  ../../milestone-7/src/native_model_wire/protocol.c
)
target_include_directories(native_semantic_basic PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../milestone-7/src"
)
target_compile_options(native_semantic_basic PRIVATE -Wall -Wextra -Wno-unused-parameter -fvisibility=hidden)
target_link_libraries(native_semantic_basic PRIVATE QNX::socket)

add_library(native_semantic_controller SHARED
  controller_addon.c
  semantic_ctrl_wire.c
  semantic_handoff.c
  semantic_minimap_shm.c
)
target_include_directories(native_semantic_controller PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../milestone-7/src"
)
target_compile_options(native_semantic_controller PRIVATE -Wall -Wextra -Wno-unused-parameter -fvisibility=hidden)
target_link_libraries(native_semantic_controller PRIVATE QNX::socket)

add_library(nav_addon_loader SHARED
  nav_addon_loader.c
)
target_compile_options(nav_addon_loader PRIVATE -Wall -Wextra -Wno-unused-parameter -fvisibility=hidden)

install(TARGETS nav_addon_loader
  LIBRARY DESTINATION share/nbt-aad/nav-addons)
install(TARGETS native_semantic_basic
  LIBRARY DESTINATION share/nbt-aad/nav-addons/NavigationBasic.d)
install(TARGETS native_semantic_controller
  LIBRARY DESTINATION share/nbt-aad/nav-addons/NavNBTEvoController.d)
