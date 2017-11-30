#!/bin/sh

# Copyright (c) 2017, PlusOne Robotics Inc.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description
# CI using industrial_ci runs on Docker image where a few necessary settings are missing.
export RUNLEVEL=3
echo "This is only required for the docker container"
echo "Update content of policy-rc.d so that codemeter can be started"
printf "#!/bin/sh\nexit 0" > /usr/sbin/policy-rc.d
# Install optional packages to avoid warnings during compilation
apt-get install libopenni2-dev libpcap-dev libpng12-dev -y

# Run installation of ensenso proprietary DEBs
./scripts/install_driver.sh

# Need to patch /opt/ensenso/cmake/FindEnsenso.cmake
FILE_TO_PATCH="/opt/ensenso/cmake/FindEnsenso.cmake"
/bin/cat <<EOM >$FILE_TO_PATCH
# - Try to find EnsensoSDK
# Once done this will define
#  ENSENSO_FOUND
#  ENSENSO_INCLUDE_DIRS
#  ENSENSO_LIBRARIES
#  ENSENSO_DEFINITIONS
find_package(PkgConfig)
pkg_check_modules(PC_ENSENSO QUIET ensenso-nxlib)
set(ENSENSO_DEFINITIONS \${PC_ENSENSO_CFLAGS_OTHER})
find_path(ENSENSO_INCLUDE_DIR nxLib.h
          HINTS \${PC_ENSENSO_INCLUDEDIR} \${PC_ENSENSO_INCLUDE_DIRS} "/opt/ensenso/development/c/include")
find_library(ENSENSO_LIBRARY NAMES NxLib32 NxLib64
             HINTS \${PC_ENSENSO_LIBDIR} \${PC_ENSENSO_LIBRARY_DIRS})
set(ENSENSO_LIBRARIES \${ENSENSO_LIBRARY} )
set(ENSENSO_INCLUDE_DIRS \${ENSENSO_INCLUDE_DIR} )
include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ENSENSO_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Ensenso DEFAULT_MSG
                                  ENSENSO_LIBRARY ENSENSO_INCLUDE_DIR)
mark_as_advanced(ENSENSO_INCLUDE_DIR ENSENSO_LIBRARY )
EOM
