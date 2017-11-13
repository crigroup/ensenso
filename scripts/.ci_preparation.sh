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
echo "Update content of policy-rc.d."; printf "#!/bin/sh\nexit 0" > /usr/sbin/policy-rc.d

# Run installation of ensenso proprietary DEBs
./install_driver.sh
