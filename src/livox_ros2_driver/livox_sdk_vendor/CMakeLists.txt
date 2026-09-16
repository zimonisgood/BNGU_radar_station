cmake_minimum_required(VERSION 3.9)

project(livox_sdk_vendor)

find_package(ament_cmake REQUIRED)

macro(build_livox_sdk)
  set(livox_sdk_REV "v2.3.1")
  set(extra_cmake_args)

  # SDK 2.3.1 bundles fmt with a pre-C++20 char8_t type. GCC 13 warns
  # about this valid C++11 code, and the SDK promotes warnings to errors.
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
     CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
    list(APPEND extra_cmake_args
      "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS} -Wno-error=c++20-compat")
  endif()

  get_property(multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(NOT multi_config AND DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  include(ExternalProject)
  externalproject_add(livox-sdk-${livox_sdk_REV}
    GIT_REPOSITORY https://github.com/Livox-SDK/Livox-SDK.git
    GIT_TAG ${livox_sdk_REV}
    GIT_SHALLOW OFF
    TIMEOUT 60
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/livox-sdk_install
      -DCMAKE_POSITION_INDEPENDENT_CODE=ON
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/livox-sdk_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}
  )
endmacro()

option(BUILTIN_livox_sdk "Force built-in version of livox-sdk" OFF)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")

find_package(livox_sdk QUIET)

if(NOT livox_sdk_FOUND OR BUILTIN_livox_sdk)
  build_livox_sdk()
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS livox_sdk_vendor-extras.cmake)
