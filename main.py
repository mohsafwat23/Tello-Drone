# MIT License

# Copyright (c) 2018 DAMIÀ FUENTES ESCOTÉ

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from djitellopy import tello
from time import sleep

me = tello.Tello()#initialze drone 
me.connect()#connect to the drone
print(me.get_battery())
#me.takeoff() #take of the drone
#me.send_rc_control(0,50,0,0)#foward velocity at 50 cm/s
#sleep(2)#delay 2 seconds
#me.send_rc_control(0, 0, 20, 0)
#sleep(1)
#me.send_rc_control(0, 0, 0, 0)
#foward velocity at 50 cm/s
#me.send_rc_control(0,-50,0,0)
sleep(5)
#me.land()
