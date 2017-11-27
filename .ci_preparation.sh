#!/bin/sh
echo "This is only required for the docker container"
echo "Update content of policy-rc.d so that codemeter can be started"
printf "#!/bin/sh\nexit 0" > /usr/sbin/policy-rc.d
# Install optional packages to avoid warnings during compilation
apt-get install libopenni2-dev libpcap-dev libpng12-dev -y
# Install the driver
./scripts/install_driver.sh
echo "List ensenso-sdk files"
dpkg -L ensenso-sdk
