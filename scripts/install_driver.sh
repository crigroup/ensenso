#!/bin/sh
# Description
# Downloading proprietary deb files from https://www.ensenso.com/support/sdk-download, and install them.

# Update the URL when newer versions available.
URL_CODEMETER=https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb
URL_ENSENSO=https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.0.147-x64.deb

PATH_DEB_CODEMETER=/tmp/codemeter.deb
PATH_DEB_ENSENSO=/tmp/ensenso.deb

wget -O $PATH_DEB_CODEMETER $URL_CODEMETER
wget -O $PATH_DEB_ENSENSO $URL_ENSENSO

for path in $PATH_DEB_CODEMETER $PATH_DEB_ENSENSO
do
  dpkg --configure -a --force-depends
  apt-get -f -y install
  dpkg -i $path
done
