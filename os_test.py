import os

os.system('roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"')
print(123456)