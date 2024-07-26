import rclpy
import os
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import face_recognition
import PIL
from PIL import ImageFont,ImageDraw
from rclpy.node import Node
from uucar_interfaces.msg import FaceData,FaceResults

class FaceTopic(Node):
    def __init__(self):
        super().__init__('face_rec_topic')
        self.known_face_encodings = list()
        self.known_face_names = list()
        ##############
        self.tolerance = 0.5 # 误差阈值
        self.face_data = '/home/xiao/uucar_ws/src/face_rec/face_data'
        self.face_load()
        ##############
        self.pub_data = self.create_publisher(FaceResults,'/face_results',10)
        self.pub_img = self.create_publisher(Image,'/camera/face_recognition', 10)
        self.sub_img = self.create_subscription(Image, '/camera1/image_raw', self.callback_img,10)

    def callback_img(self, image):
        bridgr = CvBridge()
        # 将消息转为bgr格式
        frame = bridgr.imgmsg_to_cv2(image,'bgr8')
        face_locations = [] # 检测到的未知人脸列表
        face_encodings = [] # 未知人脸编码列表
        face_names = [] # 实时标签列表列表
        process_this_frame = True 
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
                matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding,self.tolerance)
                # 得到误差最小的人脸在已知列表中的位置
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]: # 如果对比结果为True
                    name = self.known_face_names[best_match_index] # 为检测到的人脸做标签
                face_names.append(name)

        process_this_frame = not process_this_frame

        # 定义发布消息
        results = FaceResults()
        results.num = len(face_names)

        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # 在面部画一个框，放大备份人脸位置，因为我们检测到的帧被缩放到1/4大小
            top *= 4
            right *=4
            bottom *=4
            left *=4
            cv2.rectangle(frame, (left,top),(right,bottom),(0,0,255),2)
            # 在人脸下方写上标签
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            # cv2.putText(frame,name,(left - 10, bottom - 10), font, 1.0,(255,255,255), 1)
            frame = self.paint_chinese_opencv(frame, name, (left + 6, bottom - 40), (255,255,255))

            # 定义发布消息
            data = FaceData()
            data.name = name
            data.xmin = float(left)
            data.xmax = float(right)
            data.ymin = float(top)
            data.ymax = float(bottom)
            results.face_data.append(data)

        img = bridgr.cv2_to_imgmsg(frame,"bgr8") # 把OpenCV图像转换为ROS消息
        self.pub_data.publish(results)
        self.pub_img.publish(img) # 发布图像到话题
    def face_load(self):
        """加载图像并学习如何识别它，并添加到已知人脸库"""
        for name in os.listdir(self.face_data):
            self.get_logger().info("添加'%s'的人脸数据"%(name))
            file = os.path.join(self.face_data,name)
            for img in os.listdir(file):
                new_image = face_recognition.load_image_file(os.path.join(file,img)) # 将图片转换为numpy数组
                new_face_encoding = face_recognition.face_encodings(new_image)[0] # 得到人脸编码
                self.known_face_encodings.append(new_face_encoding) # 添加到已知人脸库
                self.known_face_names.append(name) # 添加人脸名称

    def paint_chinese_opencv(self,im,chinese,pos,color):
        img_PIL = PIL.Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
        font = ImageFont.truetype('NotoSansCJK-Bold.ttc',30)
        fillColor = color #(255,0,0)
        position = pos #(100,100)
        draw = ImageDraw.Draw(img_PIL)
        draw.text(position,chinese,font=font,fill=fillColor)
    
        img = cv2.cvtColor(np.asarray(img_PIL),cv2.COLOR_RGB2BGR)
        return img

def main(args=None):

    # 2.初始化 ROS2 客户端；
    rclpy.init(args=args)

    # 4.调用spin函数，并传入节点对象；
    Progress_action_server = FaceTopic()
    rclpy.spin(Progress_action_server)

    # 5.释放资源。
    rclpy.shutdown()

if __name__ == '__main__':
    main()
