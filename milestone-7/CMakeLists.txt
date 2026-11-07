# SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
# Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
# This is the authoritative target source list; no qcc/project files are read.
add_executable(nbt-aad
  src/main.cpp
  src/audio/a2i_audio.cpp
  src/audio/config.cpp
  src/audio/bt_audio.cpp
  src/audio/speech_out.cpp
  src/audio/mic_uplink.cpp
  src/audio/a2i_stream.cpp
  src/audio/audio_route.cpp
  src/audio/mixer.cpp
  src/audio/playout.cpp
  src/audio/drift.cpp
  src/audio/stream_rate.cpp
  src/audio/browser_audio.cpp
  src/audio/pcm_ring.cpp
  src/audio/resampler.cpp
  src/audio/asound_device.cpp
  src/video_manager.cpp
  src/mmr_launch.cpp
  src/tcp_server.cpp
  src/usb_accessory.cpp
  src/usb_direct.cpp
  src/aa_session.cpp
  src/audio/audio_acks.cpp
  src/aa_cryptor.cpp
  src/bluetooth_manager.cpp
  src/wireless_setup.cpp
  src/touch_probe.cpp
  src/dsi_client.cpp
  src/audio/voice_scenario.cpp
  src/audio/bt_play.cpp
  src/most/client.cpp
  src/most/trace.cpp
  src/car/ccs_input.cpp
  src/car/volume_input.cpp
  src/car/input_activity.cpp
  src/car/geoid.cpp
  src/car/gps.cpp
  src/sdr_builder.cpp
  src/crash_handler.cpp
  src/kombi_client.cpp
  src/kombi_nav.cpp
  src/volume_osd.cpp
  src/audio/speex.c
)
target_link_libraries(nbt-aad PRIVATE NBT::a2i_client QNX::socket QNX::ssl QNX::crypto QNX::screen QNX::usbdi)
add_executable(test_display test_display.cpp)
target_link_libraries(test_display PRIVATE QNX::screen)
add_executable(test_controls test_controls.cpp src/dsi_client.cpp)
add_executable(wfd_probe tools/wfd_probe.cpp)
target_link_libraries(wfd_probe PRIVATE QNX::socket)
add_executable(pf_aa tools/pf_aa.cpp)
add_executable(plugin_check plugin/plugin_check.cpp)
add_library(hmiaad SHARED plugin/nbt_aad_plugin.cpp plugin/hmi_services.cpp plugin/hmi_menu.cpp)
add_library(NbtAad SHARED rhmiapp/nbt_aad_app.cpp)
target_link_libraries(NbtAad PRIVATE NBT::AppFramework.A NBT::CoreFramework.A QNX::socket)
set_target_properties(hmiaad PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugin")
set_target_properties(NbtAad PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rhmiapp")
foreach(target nbt-aad test_display test_controls wfd_probe pf_aa plugin_check hmiaad NbtAad)
  target_include_directories(${target} PRIVATE include src)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wno-unused-parameter)
endforeach()
install(TARGETS nbt-aad test_display test_controls wfd_probe pf_aa plugin_check RUNTIME DESTINATION bin)
install(TARGETS hmiaad LIBRARY DESTINATION share/nbt-aad/hmiplugins)
install(TARGETS NbtAad LIBRARY DESTINATION share/nbt-aad/rhmiapp)
install(FILES rhmiapp/NbtAad.manifest rhmiapp/NbtAad.xml rhmiapp/NbtAad_Direct.xml
  rhmiapp/NbtAad_Multimedia.xml rhmiapp/NbtAad_ALL_Texts.zip rhmiapp/NbtAad_ALL_Images.zip
  DESTINATION share/nbt-aad/rhmiapp)
install(FILES plugin/NbtAadPlugin.xml DESTINATION share/nbt-aad/hmiplugins)
install(DIRECTORY plugin/res/ DESTINATION share/nbt-aad/hmiplugins/res PATTERN "*.py" EXCLUDE)
install(FILES third_party/speexdsp/COPYING DESTINATION share/licenses/nbt-aad/speexdsp)
