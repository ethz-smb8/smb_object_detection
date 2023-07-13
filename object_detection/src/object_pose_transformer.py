import rospy
import rospkg

import message_filters as mf

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Quaternion


from csv import writer
from datetime import datetime
import tf2_geometry_msgs
import tf2_ros
import cv2
import time
from cv_bridge import CvBridge




class SaverNode:
    def __init__(self):
        self.rospack = rospkg.RosPack()

        # Initilized the node 
        rospy.init_node("object_pose_transformer", anonymous=True)

        # ---------- Node Related Params ---------- 
        # -> Subscribed Topics
        self.detections_image_topic ='/object_detector/detections_in_image'
        self.object_poses_topic = '/object_detector/object_poses'
        self.detection_info_topic = '/object_detector/detection_info'

        self.detections_image_sub = mf.Subscriber(self.detections_image_topic, Image,  queue_size=10)
        self.object_poses_sub = mf.Subscriber(self.object_poses_topic, PoseArray, queue_size=10)
        self.detection_info_sub = mf.Subscriber(self.detection_info_topic, PoseArray, queue_size=10)
        
        queue_size=10
        sync_slop = 0.05
        self.synchronizer= mf.ApproximateTimeSynchronizer([ self.detections_image_sub,
                                                           self.object_poses_sub,
                                                           self.detection_info_sub], 
                                                          queue_size, sync_slop)

        self.imagereader = CvBridge()


        
         # ----- additional saving (team 8)------
        self.team8_counter = 0
        self.output_path='/home/team8/challenge_data/'
        self.csv_file_path = self.output_path + str(datetime.now().strftime("%Y%m%d-%H%M%S")) + '.csv'
        with open(self.csv_file_path, 'w') as csv_file: 
            writer_object = writer(csv_file)
            header_list = ['timestamp', 'counter', 'class', 'x', 'y', 'z', 'confidence', 'x_camera', 'y_camera', 'z_camera']
            writer_object.writerow(header_list)
            csv_file.close()
        
        self.csv_file = open(self.csv_file_path, 'a')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


    def convert_pose_to_map_frame(self, input_pose, stamp):       
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = self.optical_frame_id
        pose_stamped.header.stamp = stamp

        # msf_frame_id = "world_graph_msf"
        map_frame_id = "map_o3d"

        output_pose_stamped = self.tf_buffer.transform(pose_stamped, map_frame_id, rospy.Duration(1.0))

        # output_pose_stamped.header.stamp = rospy.Time(0)
        # output_pose_stamped = self.tf_buffer.transform(output_pose_stamped, map_frame_id, rospy.Duration(1.0))

        return output_pose_stamped.pose

    def prepare_and_save_data(object_poses_in_frame, object_detection_image, detection_infos):
        raw_image_stamp = object_detection_image.header.stamp
        # save the image if object detected
        if len(object_poses_in_frame) > 0:
            object_class_id = detection_infos[i].class_id
            object_confidence = detection_infos[i].confidence

            self.team8_counter += 1
            filename = self.output_path +str(raw_image_stamp)+"_"+str(self.team8_counter).zfill(6)+".jpg"
            # transform the image msg to numpy array
            cv_image = self.imagereader.imgmsg_to_cv2(object_detection_image, "bgr8")
            cv2.imwrite(filename, cv_image)
            writer_object = writer(self.csv_file)
            for i in range(len(object_poses_in_frame)):
                # ['timestamp','counter', 'class', 'x', 'y', 'z', 'confidence']
                try:
                    pose_in_final_frame = self.convert_pose_to_map_frame(object_poses_in_frame[i], raw_image_stamp)
                except:
                    pose_in_final_frame = Pose()
                    pose_in_final_frame.position.x = 0
                    pose_in_final_frame.position.y = 0
                    pose_in_final_frame.position.z = 0
                data = [raw_image_stamp, self.team8_counter, object_class_id, 
                        pose_in_final_frame.position.x, pose_in_final_frame.position.y, pose_in_final_frame.position.z,
                        object_confidence,
                        object_poses_in_frame[i].position.x, object_poses_in_frame[i].position.y, object_poses_in_frame[i].position.z] 
                writer_object.writerow(data)

    def run(self):
        def callback(detection_image_msg, object_poses_msg, detection_info_msg):
            callback_start = time.time()
            if len(object_poses_msg) > 0: # if some object detected
                self.prepare_and_save_data(object_poses_in_frame=object_poses_msg,
                                            object_detection_image=detection_image_msg,
                                            detection_infos=detection_info_msg)
                    
                    
if __name__ == '__main__':
    node = SaverNode()
    rospy.loginfo("[ObjectDetection Node] Detection has started")
    node.run()

