arduino-cli compile --fqbn arduino:avr:leonardo "$1/$1.ino" &&
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:leonardo "$1/$1.ino" --verbose
