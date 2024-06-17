import cv2
import threading

class Arducam():
    def __init__(self):

        self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
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


    def refresh(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame
    
    def get_frame(self):
        with self.lock:
            return self.frame

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()

# 사용 예시
if __name__ == '__main__':
    arducam = Arducam()

    while True:
        frame = arducam.get_frame()
        if frame is not None:
            cv2.imshow('Arducam Image', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    arducam.release()
    cv2.destroyAllWindows()