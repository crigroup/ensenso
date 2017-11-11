#!/bin/sh

# Downloading proprietary deb files from https://www.ensenso.com/support/sdk-download, and install them.
# When using this locally you may need sudo.
PATH_DEB_CODEMETER=/tmp/codemeter.deb
PATH_DEB_ENSENSO=/tmp/ensenso.deb
curl -o $PATH_DEB_CODEMETER "https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb"  # Using pipe to pass the downloaded file could cause a problem as pipe is asynchronous https://stackoverflow.com/questions/32135523/how-to-make-pipe-run-sequentially. So run dpkg in separate lines.
curl -o $PATH_DEB_ENSENSO "https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.0.147-x64.deb"
dpkg --configure -a --force-depends
dpkg -i $PATH_DEB_CODEMETER
apt-get -f -y install
dpkg -i $PATH_DEB_ENSENSO
