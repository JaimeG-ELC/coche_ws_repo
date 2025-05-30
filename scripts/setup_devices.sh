# Norma para el VESC
touch /etc/udev/rules.d/99-vesc.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' | sudo tee /etc/udev/rules.d/99-vesc.rules > /dev/null
touch /etc/udev/rules.d/99-joystick.rules
echo 'SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="0b12", MODE="0666", GROUP="dialout", SYMLINK+="sensors/joystick"' | sudo tee /etc/udev/rules.d/99-joystick.rules > /dev/null
touch /etc/udev/rules.d/99-hokuyo.rules
echo 'SUBSYSTEM=="net", ACTION=="add", ATTRS{address}=="48:b0:2d:7a:6f:6c", MODE="0666", GROUP="dialout", SYMLINK+="sensors/hokuyo"' | sudo tee /etc/udev/rules.d/99-hokuyo.rules > /dev/null


# Cargamos las nuevas reglas y las aplicamos
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER