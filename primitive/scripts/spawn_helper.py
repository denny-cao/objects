import rospy
import rospkg
import tf2_ros
import xacro
from geometry_msgs.msg import Vector3, Pose, TransformStamped, Point, Quaternion
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest, DeleteModel, DeleteModelRequest, SpawnModel, SpawnModelRequest, SpawnModelResponse

class sim_control_handler():
    def __init__(self):
        self.primitives = []
        self.primitive_model_name = None

        # Proxies
        self.delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model_proxy = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        self.primitive_spawned = False

        self.primitive_tf = TransformStamped()
        self.primitive_tf.header.frame_id = "/world" 
    
        self.primitive_tf_bcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.primitive_pose = None
        
        # Get xacro file
        rospack = rospkg.RosPack()
        self.primitive_urdf_path = rospack.get_path("franka_description") + "/robots/shape.urdf.xacro"

        # Requests
        self.spawn_model_req = SpawnModelRequest() 
        self.spawn_model_req.reference_frame = ""
        self.delete_model_req = DeleteModelRequest() 

    def update_shape(self, shape, number): 
        # Set name
        self.primitive_model_name = str(number) + "_" + shape.__class__.__name__
        self.primitives.append(self.primitive_model_name)

        # Set child frame
        self.primitive_tf.child_frame_id = self.primitive_model_name
        
        # Set position of object
        pos = Point(x=shape.x, y=shape.y, z=0)
        ori = Quaternion(x=0, y=0, z=0, w=1)
        self.primitive_pose = Pose(position=pos, orientation=ori)
        self.primitive_tf.transform.rotation = ori

    def pub_primitive_tf(self):
        self.primitive_tf_bcaster.sendTransform(self.primitive_tf)

    def delete_models(self):
        for shape in self.primitives:
            rospy.loginfo("deleting shape...")
            self.delete_model_req.model_name = shape
            self.delete_model_proxy(self.delete_model_req)
            rospy.loginfo("finished deleting shape")

        self.primitives = []
        self.primitive_spawned = False

    def spawn_model(self):
        rospy.loginfo("spawning shape...")
        
        # Parse xacro
        xacro_file = "shape.urdf.xacro"
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
        description_xml = doc.toxml()
        self.spawn_model_req.model_xml = description_xml

        self.spawn_model_req.model_name = self.primitive_model_name 
        self.spawn_model_req.initial_pose = self.primitive_pose
        
        self.spawn_model_proxy(self.spawn_model_req)

        rospy.loginfo("finished spawning shape")

        self.primitive_spawned = True
        self.pub_primitive_tf()

    def pause_sim(self):
        rospy.loginfo("PAUSING service found...")
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self.max_retry:
                try:
                    rospy.loginfo("PAUSING service calling...")
                    self.pause_proxy()
                    paused_done = True
                    rospy.loginfo("PAUSING service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/pause_physics service call failed")
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo pause service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.loginfo("PAUSING FINISH")

    def unpause_sim(self):
        rospy.loginfo("UNPAUSING service found...")
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self.max_retry:
                try:
                    rospy.loginfo("UNPAUSING service calling...")
                    self.unpause_proxy()
                    unpaused_done = True
                    rospy.loginfo("UNPAUSING service calling...DONE")
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr("/gazebo/unpause_physics service call failed...Retrying "+str(counter))
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo unpause service"
                rospy.logerr(error_message)
                assert False, error_message

        rospy.loginfo("UNPAUSING FiNISH")
