import grpc
import cv2
import sys
import os

# 프로젝트 루트 경로를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import common.proto.image_service_pb2 as image_service_pb2
import common.proto.image_service_pb2_grpc as image_service_pb2_grpc

print("ASDf")
def capture_image_from_arducam():
    # Arducam으로부터 이미지 캡처 (예시)
    cap = cv2.VideoCapture(1, cv2.CAP_V4L2)  # 카메라 ID를 0으로 설정
    cap.set(cv2.CAP_PROP_FPS, 20)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ret, frame = cap.read()
    # cap.release()
    return frame

def run():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = image_service_pb2_grpc.ImageServiceStub(channel)
        cap = cv2.VideoCapture(1, cv2.CAP_V4L2)  # 카메라 ID를 0으로 설정
        cap.set(cv2.CAP_PROP_FPS, 20)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while True:
            ret, image = cap.read()
            # image = capture_image_from_arducam()
            
            # 이미지를 화면에 표시
            # cv2.imshow('Arducam Image', image)
            # cv2.waitKey(1)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
            
            _, img_encoded = cv2.imencode('.jpg', image)
            image_data = img_encoded.tobytes()
            
            response = stub.ProcessImage(image_service_pb2.ImageRequest(image_data=image_data))
            print(f'Result: {response.result}')
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    run()
