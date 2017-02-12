#!/usr/bin/env python

""" 
  A simple Controller GUI to drive robots and pose heads.
  Copyright (c) 2008-2011 Michael E. Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  Modifications by Patrick Goebel for the Pi Robot Project
"""

import roslib; #roslib.load_manifest('pi_head_tracking_3d_part1')
import rospy
import wx

import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

width = 300
height = 200

class PointHeadGUI(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, debug = False):  
        wx.Frame.__init__(self, parent, -1, "Point Head GUI", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))

        # Intialize the transform listener
        self.tf = tf.TransformListener()
        
        # Make sure at least the head_pan and head_tilt frames are visible
        #self.tf.waitForTransform('head_pan_link', 'head_tilt_link', rospy.Time(), rospy.Duration(5.0))
        
        # Get the list of all frames
        self.frames = self.tf.getFrameStrings()
        
        # Initialize the target point
        self.target_point = PointStamped()

        # Create the target point publisher
        self.targetPub = rospy.Publisher('/target_point', PointStamped)

        sizer = wx.GridBagSizer(15,10)

        # Select Target Reference Frame
        selectFrame = wx.StaticBox(self, -1, 'Select Frame')
        selectFrame.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        selectFrameBox = wx.StaticBoxSizer(selectFrame, orient=wx.VERTICAL) 
        frameSizer = wx.GridBagSizer(3, 10)
        self.frameComboBox = wx.ComboBox(self, -1, style=wx.CB_READONLY)
        self.Bind(wx.EVT_COMBOBOX, self.on_combobox, self.frameComboBox)
        self.update_combobox()
        frameSizer.Add(self.frameComboBox, (0, 0))   
        selectFrameBox.Add(frameSizer)
        sizer.Add(selectFrameBox,(0,0),wx.GBSpan(3,10),wx.EXPAND|wx.TOP|wx.RIGHT|wx.LEFT,5)

        # Select x y z target coordinates relative to the frame selected above
        selectPoint = wx.StaticBox(self, -1, 'Set Target Point')
        selectPoint.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        selectPointBox = wx.StaticBoxSizer(selectPoint, orient=wx.VERTICAL) 
        headSizer = wx.GridBagSizer(3, 10)
        headSizer.Add(wx.StaticText(self, -1, "x:"),(0,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.target_point_x = wx.TextCtrl(self,-1,value=u"0.0")
        headSizer.Add(self.target_point_x, (0,1))
        headSizer.Add(wx.StaticText(self, -1, "y:"),(1,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.target_point_y = wx.TextCtrl(self,-1,value=u"0.0")
        headSizer.Add(self.target_point_y, (1,1))
        headSizer.Add(wx.StaticText(self, -1, "z:"),(2,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.target_point_z = wx.TextCtrl(self,-1,value=u"0.0")
        headSizer.Add(self.target_point_z, (2,1))
        selectPointBox.Add(headSizer) 
        sizer.Add(selectPointBox, (3,0), wx.GBSpan(3,10), wx.EXPAND|wx.BOTTOM|wx.RIGHT|wx.LEFT,5)

        # Point head button
        pointSizer = wx.GridBagSizer(1, 10)
        self.pointButton = wx.Button(self, -1, label="Point Head!")
        self.Bind(wx.EVT_BUTTON, self.on_point_button, self.pointButton)
        pointSizer.Add(self.pointButton, (0, 1))   
        
        # Reset head button
        self.resetButton = wx.Button(self, -1, label="Reset Position")
        self.Bind(wx.EVT_BUTTON, self.on_reset_button, self.resetButton)
        pointSizer.Add(self.resetButton, (0, 2))   
        sizer.Add(pointSizer, (6,0), wx.GBSpan(2,10), wx.EXPAND|wx.BOTTOM|wx.RIGHT|wx.LEFT,5)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(50)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        # bind the panel to the paint event
        wx.EVT_PAINT(self, self.onPaint)
        self.dirty = 1
        self.onPaint()

        self.SetSizerAndFit(sizer)
        self.Show(True)

    def update_combobox(self):
        self.frameComboBox.SetItems(self.frames)
        selected_frame = self.get_selected_frame()

    def get_selected_frame(self):
        return str(self.frameComboBox.GetValue())

    def on_combobox(self, event):
        self.target_point.header.frame_id = self.get_selected_frame().rstrip()

    def on_point_button(self, event):
        try:
            self.target_point.header.frame_id = 'kinect2_link'
            self.target_point.point.x = float(self.target_point_x.GetValue().rstrip())
            self.target_point.point.y = float(self.target_point_y.GetValue().rstrip())
            self.target_point.point.z = float(self.target_point_z.GetValue().rstrip())
            rospy.loginfo("Publishing target point:\n" + str(self.target_point))
            self.targetPub.publish(self.target_point)
        except:
            rospy.loginfo("Invalid floating point coordinates.  Please try again.")
            
    def on_reset_button(self, event):
        # Set the head level and straight ahead but setting a far away target point relative to the base_link frame.
        try:
            self.target_point.point.x = 1000.
            self.target_point.point.y = 0.0
            self.target_point.point.z = 0.0
            self.target_point.header.frame_id = '/base_link'
            rospy.loginfo("Publishing target point:\n" + str(self.target_point))
            self.targetPub.publish(self.target_point)
        except:
            rospy.loginfo("Invalid floating point coordinates.  Please try again.")

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def onPaint(self, event=None):
        pass

    def onTimer(self, event):
        if rospy.is_shutdown():
            self.Close(True)
            self.Refresh()

    def onExit(self, e):
        self.Close(True)
        self.Refresh()

    def onError(self):
        self.Raise()

if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('point_head_gui')
    app = wx.PySimpleApp()
    frame = PointHeadGUI(None, True)
    app.MainLoop()

