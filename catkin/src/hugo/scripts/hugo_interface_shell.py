#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
import sys
from std_msgs.msg import Int16

class Hugo_interface_shell:
    
    def __init__(self, node_name_override = 'hugo_interface_shell'):
        
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        rospy.loginfo("hugo_interface_shell starting with name %s", self.nodename)
        
        self.freq = 5 
        self.rate = rospy.get_param("hugo_interface_shell with freq", self.freq)

        

        self.audioPub = rospy.Publisher("audio", Int16)
        self.bolitaPub = rospy.Publisher("bolita", Int16)
        self.videoPub  = rospy.Publisher("video" , Int16)
        #self.leftPub   = rospy.Publisher("left"  , Int16) # CONTINUO!
        #self.rightPub  = rospy.Publisher("right" , Int16) # CONTINUO!

    def update(self):

        #self.audioPub.publish(self.audio_lock);
        #self.bolitaPub.publish(self.bolita_lock);
        #self.videoPub.publish(self.video_lock);
         
        val = raw_input()

        self.audio_lock = 0
        self.bolita_lock = 0
        self.video_lock = 0

        self.audioPub.publish(self.audio_lock);
        self.bolitaPub.publish(self.bolita_lock);
        self.videoPub.publish(self.video_lock);

        
        if (val is not None and val is not ""):
            
            vals = val.split()
            
            if (len(vals) >= 1):
                if (vals[0] == "^C"):
                    sys.exit(0)
                
            if (len(vals) >= 1):
                try:

                    self.char1 = vals[0]
                    
                    if (self.char1 == 'b'):
                        self.bolita_lock = 1
                        #return

                    if (self.char1 == 'v'):
                        self.video_lock = 1
                        #return

                    if (len(vals) == 2):

                        if (self.char1 == 'a'):
                            self.char2 = vals[1]
                            self.audio_lock = int(self.char2)
                            #return

                except:
                    pass

        self.audioPub.publish(self.audio_lock);
        self.bolitaPub.publish(self.bolita_lock);
        self.videoPub.publish(self.video_lock);

        self.audio_lock = 0
        self.bolita_lock = 0
        self.video_lock = 0

        self.audioPub.publish(self.audio_lock);
        self.bolitaPub.publish(self.bolita_lock);
        self.videoPub.publish(self.video_lock);

    def spin(self):
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            self.update()
            r.sleep()

if __name__ == '__main__':
    """ main """
    hugo_interface_shell = Hugo_interface_shell()
    hugo_interface_shell.spin()
    
