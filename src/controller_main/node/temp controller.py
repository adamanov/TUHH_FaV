
        # Subscribe to topics of the controller output
        rospy.Subscriber("thrust/control_effort",
                         Float64,
                         self.control_callback_thrust,
                         queue_size=1)
        rospy.Subscriber("vertical_thrust/control_effort",
                         Float64,
                         self.control_callback_vertical_thrust,
                         queue_size=1)
        rospy.Subscriber("lateral_thrust/control_effort",
                         Float64,
                         self.control_callback_lateral_thrust,
                         queue_size=1)
        rospy.Subscriber("yaw/control_effort",
                         Float64,
                         self.control_callback_yaw,
                         queue_size=1)
        rospy.Subscriber("pitch/control_effort",
                         Float64,
                         self.control_callback_pitch,
                         queue_size=1)
        rospy.Subscriber("roll/control_effort",
                         Float64,
                         self.control_callback_roll,
                         queue_size=1)

        rospy.Subscriber("orientation/euler", Orientation,
                         self.orient_callback,
                         queue_size=1)
        
# ------------------------control_callback_vertical_thrust-------------------------------------------------------
    def control_callback_vertical_thrust(self, msg):
        print(msg.data)
        self.control_effort_vertical_thrust = msg.data
        
        safezone_upper = rospy.get_param('safezone_upper')
        safezone_lower = rospy.get_param('safezone_lower')
        # print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort))
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            if (self.depth < safezone_upper and self.depth > safezone_lower):
                # Congratulations You passed ALL CHECKS !!!
                self.set_vertical_thrust(self.control_effort_vertical_thrust)
            elif self.depth > safezone_upper:
                self.set_vertical_thrust(min(self.control_effort_vertical_thrust, 0))
                rospy.loginfo(
                    "over safezone_upper!!   (No positive control_effort allowed)")
            elif self.depth < safezone_lower:
                self.set_vertical_thrust(max(self.control_effort, 0))
                rospy.loginfo(
                    "under safezone_lower!!   (No negative control_effort allowed)")
            else:
                self.set_vertical_thrust(0.0)
                rospy.loginfo(
                    "WARNING: Numerical Problems??? control_effort is 0!")
        elif isNotTimedout:
            self.set_vertical_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_vertical_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_vertical_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didn't recieved any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        


# ------------------------control_callback_thrust-------------------------------------------------------

    def control_callback_thrust(self, msg):
        print(msg.data)
        self.control_effort_thrust = msg.data
        '''
        # print("setpoint: " + str(self.depth_setpoint) + "   depth: " + str(self.depth) + "  Control_effort: " + str(self.control_effort_thrust))
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_thrust(self.control_effort_thrust)
        elif isNotTimedout:
            self.set_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        '''

# --------------------control_callback_lateral_thrust--------------------------------------------------------------------

    def control_callback_lateral_thrust(self, msg):
        print(msg.data)
        self.control_effort_lateral_thrust = msg.data
        '''
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_lateral_thrust(self.control_effort_lateral_thrust)
        elif isNotTimedout:
            self.set_lateral_thrust(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_lateral_thrust(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_lateral_thrust(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        '''


# -------------------------control_callback_yaw--------------------------------------------------------

    def control_callback_yaw(self, msg):
        print(msg.data)
        self.control_effort_yaw = msg.data
        '''
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_yaw_rate(self.control_effort_yaw)
        elif isNotTimedout:
            self.set_yaw_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_yaw_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_yaw_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        '''

# ---------------------control_callback_pitch--------------------------------------------------------------

    def control_callback_pitch(self, msg):
        print(msg.data)
        self.control_effort_pitch = msg.data
        '''
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            # Congratulations You passed ALL CHECKS !!!
            self.set_pitch_rate(self.control_effort_pitch)
        elif isNotTimedout:
            self.set_pitch_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_pitch_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_pitch_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        '''

# -----------------------control_callback_rol---------------------------------------------------------------


    def control_callback_roll(self, msg):
        print(msg.data)
        self.control_effort_roll = msg.data
        '''
        isNotTimedout = rospy.get_param(
            'depth_timeout') > rospy.Time.now().nsecs - self.last_depth_time
        if (isNotTimedout and self.isStable()):
            self.set_roll_rate(self.control_effort_roll)
            # rospy.loginfo("No limitations to control_effort_roll")
        elif isNotTimedout:
            self.set_roll_rate(0.0)
            rospy.loginfo("Wet whale is tilted!")
            rospy.loginfo("(You could change 'euler_threshold')")
        elif self.isStable():
            self.set_roll_rate(0.0)
            rospy.loginfo("No depth data recieved in the last sec! ")
            rospy.loginfo("(You could change 'depth_timeout')")
        else:
            self.set_roll_rate(0.0)
            rospy.loginfo(
                "Wet whale is tilted over and didnt recieve any depth data !")
            rospy.loginfo(
                "(You could change 'depth_timeout' and 'euler_threshold')")
        '''

