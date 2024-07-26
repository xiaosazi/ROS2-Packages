import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from uucar_interfaces.srv import RecognitionResults
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import face_recognition
import PIL
from PIL import ImageFont,ImageDraw
from uucar_interfaces.msg import FaceData,FaceResults

class FaceRecService(Node):
    def __init__(self):
        super().__init__('face_rec_service')
        self.known_face_encodings = list()
        self.known_face_names = list()
        ##############
        self.tolerance = 0.5 # 误差阈值
        # 参数读取
        self.declare_parameter("face_data","")
        # 写入参数
        self.face_data = self.get_parameter("face_data")
        # self.get_logger("rclpy").info("face_data = %s" % self.face_data.value)
        # self.face_data = '/home/xiao/uucar_ws/src/face_rec/face_data'
        self.face_load()
        self.sub_img = self.create_subscription(Image, '/camera1/image_raw', self.callback_img,10)
        self.server = self.create_service(RecognitionResults, 'face_rec', self.detect_callback)
    def callback_img(self, image):
        global img 
        img = image
    def detect_callback(self,resquest,response):
        results = FaceResults()
        if resquest.mode == 1:
            bridgr = CvBridge()
            # 将消息转为bgr格式
            frame = bridgr.imgmsg_to_cv2(img,'bgr8')
            return self.generate_srv(frame,response)
        elif resquest.mode ==2:
            print(resquest.image_path)
            frame = cv2.imread(resquest.image_path,1)
            if(frame.date==None):
                response.result = results
                response.success = False
                return response
            else:  
                return self.generate_srv(frame,response)
        else:
            response.result = results
            response.success = False
            return response
    def generate_srv(self,frame,response):
        print('-----------------------------------------------')
        print('开始检测')
        
        face_locations = [] # 检测到的未知人脸列表
        face_encodings = [] # 未知人脸编码列表
        face_names = [] # 实时标签列表列
        process_this_frame = True 
        results = FaceResults()
        # 将视频帧大小调整为1/4大小，以加快人脸识别处理速度
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        # 将图像从 BGR 颜色（OpenCV 使用）转换为 RGB 颜色（face_recognition使用）
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        if process_this_frame == True: # 间隔
            # 查找当前视频帧中的所有人脸
            face_locations = face_recognition.face_locations(rgb_small_frame)       
            # 把查找到人脸进行编码
            face_encodings = face_recognition.face_encodings(rgb_small_frame,face_locations)
            face_names = []
            self.get_logger().info("检测到的人脸数:%s"%(len(face_encodings)))
            for face_encoding in face_encodings:
                # 将检测到的人脸和已知人脸库中的图片比较
                name = "Unknown"
                # 计算检测到的人脸和已知人脸的误差
                face_distances = face_recognition.face_distance(self.known_face_encodings,face_encoding)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                # 得到误差最小的人脸在已知列表中的位置
                best_match_index = np.argmin(face_distances)
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,self.tolerance)
                if matches[best_match_index]: # 如果对比结果为True
                    name = self.known_face_names[best_match_index] # 为检测到的人脸做标签
                face_names.append(name)

        process_this_frame = not process_this_frame
            
        results.num = len(face_names)
        
        
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # 在面部画一个框，放大备份人脸位置，因为我们检测到的帧被缩放到1/4大小
            data = FaceData()
            data.name = name
            data.xmin = float(left)
            data.xmax = float(right)
            data.ymin = float(top)
            data.ymax = float(bottom)
            self.get_logger().info("检测到:%s"%(name))
            results.face_data.append(data)
        print('-----------------------------------------------')
        response.result = results
        response.success = False
        return response
    def face_load(self):
        """加载图像并学习如何识别它，并添加到已知人脸库"""
        for name in os.listdir(self.face_data.value):
            self.get_logger().info("添加'%s'的人脸数据"%(name))
            file = os.path.join(self.face_data.value,name)
            for img in os.listdir(file):
                new_image = face_recognition.load_image_file(os.path.join(file,img)) # 将图片转换为numpy数组
                new_face_encoding = face_recognition.face_encodings(new_image)[0] # 得到人脸编码
                self.known_face_encodings.append(new_face_encoding) # 添加到已知人脸库
                self.known_face_names.append(name) # 添加人脸名称

def main(args=None):
    rclpy.init(args=args)
    sevice = FaceRecService()
    rclpy.spin(sevice)
    rclpy.shutdown()

if __name__ == '__main__':
    main()