# Norma para el VESC
touch /etc/udev/rules.d/99-vesc.rules
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"' | sudo tee /etc/udev/rules.d/99-vesc.rules > /dev/null

# Cargamos las nuevas reglas y las aplicamos
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout $USER