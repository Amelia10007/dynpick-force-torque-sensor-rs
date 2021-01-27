# Setup for USB connection to WDF-6M200-3, a sensor of dynpick.
#
# This shell script must be executed in super user privileges.

echo 'Setup udev rule for dynpick force torque sensor.'
echo 'NOTE: this shell script must be executed in super user privileges.'

# Install udev, device management tool.
apt install libudev-dev

# Create an udev-customized rule.
#
# NOTE: The following product-ID may be specific for WDF-6M200-3.
mkdir -p /etc/udev/rules.d
echo 'ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"' >> /etc/udev/rules.d/80-wacoh.rules

# udev must be rebooted because we modified the rule.
service udev restart
