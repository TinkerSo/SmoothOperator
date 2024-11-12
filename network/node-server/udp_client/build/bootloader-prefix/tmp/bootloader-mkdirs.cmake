# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/ericchen/esp/esp-idf/components/bootloader/subproject"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/tmp"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/src"
  "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/ericchen/Desktop/SmoothOperator/network/node-server/udp_client/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
