#!/bin/sh

# Downloading proprietary deb files from https://www.ensenso.com/support/sdk-download, and install them.
# Lots of logics in this file are specific to Docker. When used on a local host, you will need sudo, but won't need some tweeks.

export RUNLEVEL=3
PATH_DEB_CODEMETER=/tmp/codemeter.deb
PATH_DEB_ENSENSO=/tmp/ensenso.deb

curl -o $PATH_DEB_CODEMETER "https://download.ensenso.com/s/ensensosdk/download?files=codemeter_6.40.2402.501_amd64.deb"  # Using pipe to pass the downloaded file could cause a problem as pipe is asynchronous https://stackoverflow.com/questions/32135523/how-to-make-pipe-run-sequentially. So run dpkg in separate lines.
curl -o $PATH_DEB_ENSENSO "https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.0.147-x64.deb"

echo "Content of policy-rc.d:"
cat /usr/sbin/policy-rc.d
echo "Update content of policy-rc.d."
printf "#!/bin/sh\nexit 0" > /usr/sbin/policy-rc.d

dpkg --configure -a --force-depends
apt-get -f -y install
dpkg -i $PATH_DEB_CODEMETER
dpkg --configure -a --force-depends
apt-get -f -y install
dpkg -i $PATH_DEB_ENSENSO
