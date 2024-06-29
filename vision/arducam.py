import cv2
import threading
import rclpy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Arducam():
    def __init__(self, video_num=0, publish_ros=False):

        self.cap = cv2.VideoCapture(video_num, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FPS, 20)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # self.cap.set(cv2.CAP_PROP_FOCUS, 10)  # 0부터 255까지 조절 가능, 카메라에 따라 범위가 다를 수 있음

        # 노출 시간 조정
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.1)  # 노출 시간을 설정
        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)  # 밝기를 설정
        # self.cap.set(cv2.CAP_PROP_CONTRAST, 50)  # 대비를 설정

        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        
        self.thread = threading.Thread(target=self.refresh, daemon=True)
        self.thread.start()
        self.publishing = False

        if publish_ros:
            self.node = rclpy.create_node('arducam_publisher')
            self.bridge = CvBridge()
            self.publisher = self.node.create_publisher(CompressedImage, '/compressed_image', 10)


    def refresh(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

                    if self.publishing:
                        compressed_img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
                        self.publisher.publish(compressed_img_msg)
    
    def get_frame(self):
        with self.lock:
            return self.frame

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()

    def pub_start(self):
        with self.lock:
            if not self.publishing:
                self.publishing = True
                print("Image publishing started.")
            else:
                print("Image publishing is already running.")

    def pub_stop(self):
        with self.lock:
            self.publishing = False
            print("Image publishing stopped.")
    def get_publishing(self):
        with self.lock:
            return self.publishing 

# 사용 예시
if __name__ == '__main__':
    arducam = Arducam(video_num=0,publish_ros = True)

    while True:
        frame = arducam.get_frame()
        if not arducam.get_publishing():
            arducam.pub_start()
        # if frame is not None:
        #     cv2.imshow('Arducam Image', frame)
        #     cv2.waitKey(1)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    arducam.release()
    cv2.destroyAllWindows()