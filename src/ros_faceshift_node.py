#!/usr/bin/env python  

PACKAGE='ros_faceshift'
import roslib
roslib.load_manifest(PACKAGE)

import rospy
import time
import socket
import math
import copy
import struct
import select
from ros_pololu_servo.msg import *
from ros_faceshift.msg import *

LISTENING_PORT = 33433
#BINDING_ADDR = "127.0.0.1"     # Good for local work
BINDING_ADDR = ''   # Empty string means: bind to all network interfaces

BLOCK_ID_TRACKING_STATE = 33433     # According to faceshift docs

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
    
# This applies the FaceShift channel values to the Fce control rig using the manually calibrated mapping dictionay 'fs_to_mh_control_rig_vectors'
def ApplyFSValues(target_object, bs_names, bs_vals):
    # For each channel (bs_name), look-up the mapping vector in the dictionary and add to the accumulator.
    for bs_name, bs_val in zip(bs_names, bs_vals):
        
        #Skip Non Existing Keys
        if (target_object.find(bs_name) == -1):
            continue
        target_object[bs_name].value = bs_val

class FaceShiftReceiver :
    """This is the receiving Thread listening for FaceShift UDP messages on some port."""
    
    def decode_faceshift_datastream(self, data):
        """ Takes as input the bytes of a binary DataStream received via network.
        
         If it is a Tracking State block (ID 33433) then extract some data (info, blendshapes, markers, ...).
         Otherwise None is returned.
        """
    
        #block_id = struct.unpack_from('H', data)
        #print("Received block id " + str(block_id)) ;

        offset = 0
        block_id, version, block_size = struct.unpack_from('HHI', data, offset)
     
        #print("ID, v, size = " + str(block_id) + "," + str(version) + "," + str(block_size) )
   
        offset += 8

        if(block_id == BLOCK_ID_TRACKING_STATE):
            n_blocks, = struct.unpack_from('H', data, offset)
            #print("n_blocks = " + str(n_blocks))
            offset += 2

            track_ok = 0                # Will be a byte: 1 if tracking ok, 0 otherwise.
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
                    ts, track_ok = struct.unpack_from('dB', data, offset)
                    #print("timestamp, track_ok " + str(ts) + ", " + str(track_ok) )
                    #offset += 9
                elif(block_id == 102):      # Pose block (head rotation and position)
                    x,y,z,w = struct.unpack_from('ffff', data, offset)
                    #head_rotation_quat = mathutils.Quaternion((w,x,y,z))
                elif(block_id == 103):      # Blendshapes block (blendshape values)
                    n_coefficients, = struct.unpack_from('I', data, offset)
                    #print("Blend shapes count="+ str(n_coefficients) )
                    i = 0
                    coeff_list = ""
                    while(i < n_coefficients):
                        # Offset of the block, plus the 4 bytes for int n_coefficients, plus 4 bytes per float
                        val, = struct.unpack_from('f', data, offset + 4 + (i*4))
                        blend_shape_values.append(val)
                        coeff_list += repr(val) + " "
                        i += 1
                    print("Values: " + coeff_list)
                elif(block_id == 104):     # Eyes block (eyes gaze)
                    leye_theta, leye_phi, reye_theta, reye_phi = struct.unpack_from('ffff', data, offset)
                elif(block_id == 105):     # Markers block (absolute position of mark points)
                    n_markers, = struct.unpack_from('H', data, offset)
                    #print("n markers="+str(n_markers))
                    i = 0
                    while(i < n_markers):
                        # Offset of the block, plus the 2 bytes for int n_markers, plus 4 bytes for each x,y,z floats
                        x, y, z = struct.unpack_from('fff', data, offset + 2 + (i*4*3))
                        #print("m" + str(i) + " " + str(x) + "\t" + str(y) + "\t" + str(z))
                        markers_position.append(mathutils.Vector((x,y,z)))
                        i += 1
            
                curr_block += 1
                offset += block_size
            
            msg = fsMsgTrackingState()

            msg.m_timestamp = ts

            self.pub.publish(msg)

            # end -- while on blocks. Track State scan complete

    def execute(self, context):
       
        self.report({'INFO'}, "FaceShift starting")
        
        # First try to create the socket and bind it
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

            self.namespace = rospy.get_namespace()
            rospy.init_node('faceshift_pau', anonymous=True)

            self.pub = rospy.Publisher(self.namespace + 'faceshift_pau', fsMsgTrackingState)

            while True:
                ready = select.select([self.sock], [], [], 0.5)
                if ready:
                    msg = self.sock.recv(4096)
                    self.decode_faceshift_datastream(msg)

        except OSError as msg:
            print("FaceShift thread, binding Exception: "+ str(msg))
            self.report({'ERROR'}, "FaceShift thread, binding Exception: "+str(msg))

            if(self.sock != None):
                self.sock.close()
                self.sock = None
            
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        print("FaceShift modal command, closing socket...")
        if(self.sock != None):
            self.sock.close()
            self.sock = None

        print("closed.")
        
        self.report({'INFO'}, "FaceShift exit")
        return {'CANCELLED'}
    
    def __del__(self):
        # The sock attribute might not have been defined if the command was never run
        if(hasattr(self, 'sock')):
            if(self.sock != None):
                self.sock.close()
                self.sock = None

    def report(self, msg, arg):
        print(msg)

if __name__ == "__main__":
    print("Registered FaceShift operators")

    fsr = FaceShiftReceiver()
    fsr.execute({})

    rospy.spin()
