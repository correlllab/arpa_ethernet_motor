#First Byte is enable [0,1], 2nd is speed [0,255]

#This will turn on the motor at its full speed
echo -n -e "\x01\xff" | nc -u -w1 128.138.224.6 8888

#This will turn off the motor
#echo -n -e "\x00\xff" | nc -u -w1 128.138.224.6 888



