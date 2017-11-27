#!/bin/sh
echo "Update content of policy-rc.d so that codemeter can be started"
echo "This is only required for the docker container"
printf "#!/bin/sh\nexit 0" > /usr/sbin/policy-rc.d
# Install the driver
./scripts/install_driver.sh
echo "List ensenso-sdk files"
dpkg -L ensenso-sdk
