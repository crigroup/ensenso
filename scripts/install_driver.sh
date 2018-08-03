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
# Downloading proprietary deb files from https://www.ensenso.com/support/sdk-download, and install them.
# Lots of logics in this file are specific to Docker. When used on a local host, you will need sudo, but won't need some tweeks.

# Update the URL when newer versions available.
URL_CODEMETER=https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
URL_ENSENSO=https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.2.65-x64.deb

PATH_DEB_CODEMETER=/tmp/codemeter.deb
PATH_DEB_ENSENSO=/tmp/ensenso.deb

# Downlad and install codemeter
wget -O $PATH_DEB_CODEMETER $URL_CODEMETER
dpkg -i $PATH_DEB_CODEMETER
apt-get install -f -y

# Downlad and install ensenso-sdk
wget -O $PATH_DEB_ENSENSO $URL_ENSENSO
dpkg -i $PATH_DEB_ENSENSO
apt-get install -f -y
