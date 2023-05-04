#!/usr/bin/env python3
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty

try:
    from zyre_pyzmq import Zyre as Pyre
except Exception as e:
    print("Using Python native module:", e)
    from pyre import Pyre

from pyre import zhelper
import json
import zmq
import struct

import rospy

UPDATE_RATE = 10.


class EagleROS:
    '''Class to get 2D pose estimates from Project Eagle camera system.
    '''
    def __init__(self):
        '''Constructor for EagleBridge class.
        '''
        rospy.init_node('eagle_ros')

        # Variables
        self.pose_est = Pose2D()

        self.update_rate = rospy.Rate(UPDATE_RATE)

        # self.detection_thread = threading.Thread(target=self.detect)

        # Topics
        self.pose_pub = rospy.Publisher(
            '/eagle_ros/pose_est', Pose2D, queue_size=1)

        rospy.Subscriber('/eagle_ros/calibrate', Empty, self.calibrate)

    def start(self):
        '''Main loop that listens to pyre for new measurements.
        '''
        # self.detection_thread.start()
        ctx = zmq.Context()
        chat_pipe = zhelper.zthread_fork(ctx, self.detect)

        rospy.loginfo('EAGL - Eagle Ros started')

        while not rospy.is_shutdown():
            try:
                self.pose_pub.publish(self.pose_est)
                self.update_rate.sleep()

                # msg = input()
                # print('inputtt')

                # chat_pipe.send(msg.encode('utf_8'))
            except (KeyboardInterrupt, SystemExit):
                print('KeyboardInterrupt')
                chat_pipe.send("$$STOP".encode('utf_8'))
                break

        # chat_pipe.send("$$STOP".encode('utf_8'))
        rospy.loginfo("EAGL - FINISHED")

    def detect(self, ctx, pipe):
        '''Continuously detect tags.
        '''
        n = Pyre("EAGLE")
        n.set_header("CHAT_Header1", "example header1")
        n.join("EAGLE")
        n.start()

        poller = zmq.Poller()
        poller.register(pipe, zmq.POLLIN)
        poller.register(n.socket(), zmq.POLLIN)
        while not rospy.is_shutdown():
            items = dict(poller.poll())
            if pipe in items and items[pipe] == zmq.POLLIN:
                message = pipe.recv()
                # message to quit
                if message.decode('utf-8') == "$$STOP":
                    break
                # print("CHAT_TASK: %s" % message)
                n.shouts("CHAT", message.decode('utf-8'))
            else:
                cmds = n.recv()
                msg_type = cmds.pop(0)
                msg_peer = cmds.pop(0)
                msg_name = cmds.pop(0)
                if msg_type.decode('utf-8') == "SHOUT":
                    group = cmds.pop(0)
                elif msg_type.decode('utf-8') == "ENTER":
                    headers = json.loads(cmds.pop(0).decode('utf-8'))

                if msg_type == b'ENTER':
                    rospy.loginfo("Eagle %s entered the network." % msg_name)
                elif msg_type == b'JOIN':
                    rospy.loginfo("Eagle %s joined the group." % msg_name)
                elif cmds == [b'']:
                    # print("Eagle %s sending empty message." % msg_name)
                    pass
                else:
                    # print("Eagle %s sending message." % msg_name)
                    data = cmds[0]
                    offset = 0
                    # parse data
                    while offset < len(data) and not rospy.is_shutdown():
                        # get the size of the header
                        header_size = struct.unpack('@I', data[offset:offset+4])[0]
                        # print("header_size %i \n" % header_size)
                        offset += 4
                        # print(offset)
                        # parse the header
                        h_id, h_time = struct.unpack('II', data[offset:(offset + header_size)])
                        # print("header_id %i \n" % h_id)
                        # print("header_time %i \n" % h_time)
                        offset += header_size
                        # print(offset)
                        # get the size of the data
                        data_size = struct.unpack('@I', data[offset:offset+4])[0]
                        offset += 4
                        # print(offset)
                        # get the data
                        buf = data[offset:offset+data_size]
                        # print("data_size %i \n" % data_size)
                        offset += data_size
                        # print(offset)
                        # parse the data
                        if h_id == 0:
                            # output = tag-id, x, y, z, r, p, y
                            (data_id, x, y, z, roll, pitch, yaw) = struct.unpack('Hdddddd', buf)
                            # print("source %s \t id %i \n x %f \t y %f \t z %f \n r %f \t p %f \t y %f" % (msg_name,data_id,x,y,z,roll,pitch,yaw))

                            self.pose_est.x = x
                            self.pose_est.y = y
                            self.pose_est.theta = yaw
        n.stop()

    def calibrate(self):
        '''Start Eagles calibration procedure.
        '''
        pass


if __name__ == '__main__':
    eagle_ros = EagleROS()
    eagle_ros.start()
