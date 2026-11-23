# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Production NavigationBasic and NavNBTEvoController semantic guidance addons (ARM QNX).
add_library(native_semantic_basic SHARED
  basic_addon.c
  fixture-provider.c
  semantic_cue_map.c
  semantic_handoff.c
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
)
target_include_directories(native_semantic_controller PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../milestone-7/src"
)
target_compile_options(native_semantic_controller PRIVATE -Wall -Wextra -Wno-unused-parameter -fvisibility=hidden)
target_link_libraries(native_semantic_controller PRIVATE QNX::socket)

install(TARGETS native_semantic_basic native_semantic_controller
  LIBRARY DESTINATION share/nbt-aad/native-semantic)
