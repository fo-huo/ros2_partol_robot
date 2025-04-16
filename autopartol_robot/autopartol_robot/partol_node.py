from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer  #坐标监听器
from tf_transformations import euler_from_quaternion, quaternion_from_euler #四元数转欧拉角度函数
import math #角度转弧度函数
import rclpy.time
from autopartol_interfaces.srv import SpeechText
from sensor_msgs.msg import Image #消息接口
from cv_bridge import CvBridge  #转换图像格式
import cv2  #保存图像


class PartolNode(BasicNavigator):
    def __init__(self, node_name='partol_node'):
        super().__init__(node_name)

        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.declare_parameter('img_save_path', '')
        self.initial_point_ = self.get_parameter('initial_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.img_save_path_ = self.get_parameter('img_save_path').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.speech_client_ =self.create_client(SpeechText, 'speech_text')
        self.cv_brige_ = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.img_callback, 1)

    def img_callback(self, msg):
        self.latest_img_ = msg


    def record_img(self):
        if self.latest_img_ is not None:
            pose = self.get_current_pose()
            cv_imge = self.cv_brige_.imgmsg_to_cv2(self.latest_img_)
            cv2.imwrite(
                f'{self.img_save_path_}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_imge
            )


    def get_pose_by_xyyaw(self, x, y, yaw):
        """ return PoseStamped对象 """
        pose =PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        qute = quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x = qute[0]
        pose.pose.orientation.y = qute[1]
        pose.pose.orientation.z = qute[2]
        pose.pose.orientation.w = qute[3]
        return pose

    def init_robot_pose(self):
        """ 初始化机器人位姿 """
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active() #等待导航可用
        
    def get_target_points(self):
        """ 通过参数值获取目标点的集合 """
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取目标点{index}->{x}, {y}, {yaw}')
        return points

    def nav_to_pose(self, target_point):
        """ 导航到目标点 """
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback =self.getFeedback()
            self.get_logger().info(f'剩余距离:{feedback.distance_remaining}')
        result = self.getResult()
        self.get_logger().info(f'导航结果:{result}')

    def get_current_pose(self):
        """ 获取机器人当前位置 """
        while rclpy.ok():
            try:
                tf = self.buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
                transform = tf.transform
                self.get_logger().info(
                     f'平移:{transform.translation}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
    def speech_text(self, text):
        """ 调用服务合成语音 """
        while not self.speech_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('语音合成服务未上线，等待中...')

        request =  SpeechText.Request()
        request.text = text 
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            response = future.result()
            if response.result == True:
                self.get_logger().info(f'语音合成成功{text}')
            else:
                self.get_logger().info('语音合成失败{text}')
        else:
            self.get_logger().info(f'语音合成服务响应失败')




def main():
    rclpy.init()
    partol = PartolNode()  #节点
    # rclpy.spin(partol)
    partol.speech_text('正在准备初始化位置')
    partol.init_robot_pose()
    partol.speech_text('位置初始化完成')

    while rclpy.ok():
        points =partol.get_target_points()
        for point in points:
            x, y, yaw = point[0], point[1], point[2]
            target_pose = partol.get_pose_by_xyyaw(x, y, yaw) 
            partol.speech_text(f'正在准备前往{x},{y}目标点')
            partol.nav_to_pose(target_pose)
            partol.speech_text(f'已经到达{x},{y}目标点，正在准备记录图像')
            partol.record_img()
            partol.speech_text(f'图像记录完成')
    rclpy.shutdown()
    

