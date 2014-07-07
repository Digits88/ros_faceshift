#!/usr/bin/env python

import rospy


import socket
import math

import copy

import struct

from pau2motors.msg import pau

trackMsg=pau()
LISTENING_PORT = 33433
#BINDING_ADDR = "127.0.0.1"     # Good for local work
BINDING_ADDR = ''   # Empty string means: bind to all network interfaces

BLOCK_ID_TRACKING_STATE = 33433     # According to faceshift docs

# Delay between modal timed updates when entered the modal command mode. In seconds.
UPDATE_DELAY = 0.04


# These are the names of the FaceShift control channels
blend_shape_names = [
    "EyeBlink_L",
    "EyeBlink_R",
    "EyeSquint_L",
    "EyeSquint_R",
    "EyeDown_L",
    "EyeDown_R",
    "EyeIn_L",
    "EyeIn_R",
    "EyeOpen_L",
    "EyeOpen_R",
    "EyeOut_L",
    "EyeOut_R",
    "EyeUp_L",
    "EyeUp_R",
    "BrowsD_L",
    "BrowsD_R",
    "BrowsU_C",
    "BrowsU_L",
    "BrowsU_R",
    "JawFwd",
    "JawLeft",
    "JawOpen",
    "JawChew",
    "JawRight",
    "MouthLeft",
    "MouthRight",
    "MouthFrown_L",
    "MouthFrown_R",
    "MouthSmile_L",
    "MouthSmile_R",
    "MouthDimple_L",
    "MouthDimple_R",
    "LipsStretch_L",
    "LipsStretch_R",
    "LipsUpperClose",
    "LipsLowerClose",
    "LipsUpperUp",
    "LipsLowerDown",
    "LipsUpperOpen",
    "LipsLowerOpen",
    "LipsFunnel",
    "LipsPucker",
    "ChinLowerRaise",
    "ChinUpperRaise",
    "Sneer",
    "Puff",
    "CheekSquint_L",
    "CheekSquint_R"]
    


#def talker():
#       pub = rospy.Publisher('faceshift_track', trackMsg, queue_size=10)
#       rospy.init_node('fs_ros', anonymous=True)
#       r = rospy.Rate(10) # 10hz
#       while not rospy.is_shutdown():
#           str = "hello world %s"%rospy.get_time()
#           rospy.loginfo(str)
#           pub.publish(str)
#           r.sleep()

                

class faceshiftRcv :
    """This is the receiving Thread listening for FaceShift UDP messages on some port."""
    def decode_faceshift_datastream(self, data):
        """ Takes as input the bytes of a binary DataStream received via network.
        
         If it is a Tracking State block (ID 33433) then extract some data (info, blendshapes, markers, ...).
         Otherwise None is returned.
        """
    
        # block_id = struct.unpack_from('H', data)
        # print("Received block id " + str(block_id)) ;

        offset = 0
        block_id, version, block_size = struct.unpack_from('HHI', data, offset)
     
        #print("ID, v, size = " + str(block_id) + "," + str(version) + "," + str(block_size) )
    
        offset += 8

        if(block_id == BLOCK_ID_TRACKING_STATE):
            n_blocks, = struct.unpack_from('H', data, offset)
            #print("n_blocks = " + str(n_blocks))
            offset += 2

            head_rotation_quat = None   # Will be filled with the rotation using mathutils.Quaternion
            blend_shape_values = []     # Will be a list of float in the range 0-1
            #eyes_values = None          # Will be a sequence of 4 angle values
            markers_position = []       # Will be a list of mathutils.Vector
        
            curr_block = 0
            while(curr_block < n_blocks):
                block_id, version, block_size = struct.unpack_from('HHI', data, offset)
                #print("ID, v, size = " + str(block_id) + "," + str(version) + "," + str(block_size) )
            
                # put the offset at the beginning of the block
                offset += 8
            
                if(block_id == 101):        # Frame Information blobk (timestamp and tracking status)
                    ts, self.track_ok = struct.unpack_from('dB', data, offset)
                elif(block_id == 102):      # Pose block (head rotation and position)
                    x,y,z,w = struct.unpack_from('ffff', data, offset)

                    trackMsg.m_headRotation.x = x
                    trackMsg.m_headRotation.y = y
                    trackMsg.m_headRotation.z = z
                    trackMsg.m_headRotation.w = w

                elif(block_id == 103):      # Blendshapes block (blendshape values)
                    n_coefficients, = struct.unpack_from('I', data, offset)
                    #print("Blend shapes count="+ str(n_coefficients) )
                    i = 0
                    coeff_list = ""
                    trackMsg.m_coeffs = []
                    while(i < n_coefficients):
                        # Offset of the block, plus the 4 bytes for int n_coefficients, plus 4 bytes per float
                        val, = struct.unpack_from('f', data, offset + 4 + (i*4))
                        trackMsg.m_coeffs.append(val)
                        #coeff_list += repr(val) + " "
                        i += 1
                    #print("Values: " + coeff_list)
                elif(block_id == 104):     # Eyes block (eyes gaze)
                    leye_theta, leye_phi, reye_theta, reye_phi = struct.unpack_from('ffff', data, offset)
                    trackMsg.m_eyeGazeLeftPitch=leye_theta
                    trackMsg.m_eyeGazeLeftYaw=leye_phi
                    trackMsg.m_eyeGazeRightPitch=reye_theta
                    trackMsg.m_eyeGazeRightYaw=reye_phi
                elif(block_id == 105):     # Markers block (absolute position of mark points)
                    #ignore this block for now
                    n_markers, = struct.unpack_from('H', data, offset)

                curr_block += 1
                offset += block_size
            self.updated=True

    def rcvr(self):
            try:
                msg = self.sock.recv(4096)
                #print("Received : " + str(msg))
                self.decode_faceshift_datastream(msg)

            except socket.timeout as to_msg:
                #print("We know it: " + str(to_msg))
                pass    # We know. Can happen very often
            except OSError as msg:
                # Note that we can enter this section also because we explicitly closed
                # the socket to interrupt receiving messages (see the terminate method)
                print("FaceShift thread, recv Exception: "+ str(msg))
                
                if(self.sock != None):
                    self.sock.close()
                    self.sock = None
                    
            return 

    def start_sock(self):
        try:
            print("Creating socket...")
            # The socket listening to incoming data. Its status will be always synchronized with the singleton attribute:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            #self.sock.setblocking(False)
            #self.sock.settimeout(0.1)
            #self.sock.setsockopt(level, optname, value)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1500)    # No buffer. We take the latest, if present, or nothing.
            print("Binding...")
            self.sock.bind((BINDING_ADDR, LISTENING_PORT))
            print("Bound.")
            #self.report({'INFO'}, "FaceShift Modal listening...")
        except OSError as msg:
            print("FaceShift thread, binding Exception: "+ str(msg))
            self.report({'ERROR'}, "FaceShift thread, binding Exception: "+str(msg))

            if(self.sock != None):
                self.sock.close()
                self.sock = None
            
        return


if __name__ == "__main__":
    pub = rospy.Publisher('faceshift_track', pau, queue_size=10)
    rospy.init_node('fs_ros', anonymous=True)
    r = rospy.Rate(60) # 10hz
    fs=faceshiftRcv()
    fs.start_sock()
    fs.updated=False
    fs.track_ok = 0
    while not rospy.is_shutdown():
        fs.rcvr()
        # Only succesfully tracked messages to be sent
        if (fs.updated and fs.track_ok > 0):
            pub.publish(trackMsg)
            fs.updated=False
            fs.track_ok = 0
        r.sleep()
    
