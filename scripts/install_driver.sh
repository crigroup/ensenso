#!/bin/sh
# Description
# Downloading proprietary deb files from https://www.ensenso.com/support/sdk-download, and install them.

# Update the URL when newer versions available.
URL_CODEMETER=https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
URL_ENSENSO=https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.0.147-x64.deb

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
