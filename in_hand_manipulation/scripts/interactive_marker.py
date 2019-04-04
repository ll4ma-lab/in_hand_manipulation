# Copyright (C) 2018  Balakumar Sundaralingam, LL4MA Lab, University of Utah

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


class intMarker:
    def __init__(self,init_node=False):
        if(init_node):
            rospy.init_node('interactive_marker_node')
        self.p=[]
        self.o=[]
        self.got_marker_pose=False
        self.loop=rospy.Rate(100)
        self.server=[]
    def clear_marker(self):
        self.server.clear()
        self.server.applyChanges()
    def create_marker(self,use_mesh=False,mesh='',s_name='simple_marker',m_name='my_marker'):
        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer(s_name)
        
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.scale=0.1
        
        int_marker.header.frame_id = "palm_link"
        int_marker.name = m_name
        int_marker.description = "Simple 6-DOF Control"
        
        # create a grey box marker
        box_marker = Marker()
        if(not use_mesh):
            box_marker.type = Marker.CUBE
            box_marker.scale.x = 0.01
            box_marker.scale.y = 0.01
            box_marker.scale.z = 0.01

        else:
            box_marker.type=Marker.MESH_RESOURCE
            box_marker.mesh_resource=mesh
            box_marker.scale.x = 1.0
            box_marker.scale.y = 1.0
            box_marker.scale.z = 1.0

            
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
            
        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append( box_marker )
        
        # add the control to the interactive marker
        int_marker.controls.append( box_control )
        
        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)
        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

     

    def processFeedback(self,feedback):
        self.p = feedback.pose.position
        self.o = feedback.pose.orientation
        self.got_marker_pose=True
        #print self.p
    def get_marker_pose(self):
        while(not self.got_marker_pose):
            self.loop.sleep()
        return self.p,self.o
'''
if __name__=="__main__":
    rospy.init_node("simple_marker")
    int_m=intMarker()
    #int_m.create_marker()
    # create mesh marker:
    int_m.create_marker(use_mesh=True, mesh='file:///home/bala/object_models/mustard_bottle/google_16k/nontextured.stl')
    while(not rospy.is_shutdown()):
        int_m.get_marker_pose()
'''
