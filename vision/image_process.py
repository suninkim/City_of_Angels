import grpc
import cv2
import common.proto.image_service_pb2
import common.proto.image_service_pb2_grpc


print("ASDf")
def capture_image_from_arducam():
    # Arducam으로부터 이미지 캡처 (예시)
    cap = cv2.VideoCapture(0)  # 카메라 ID를 0으로 설정
    ret, frame = cap.read()
    cap.release()
    return frame

def run():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = image_service_pb2_grpc.ImageServiceStub(channel)
        image = capture_image_from_arducam()
        
        _, img_encoded = cv2.imencode('.jpg', image)
        image_data = img_encoded.tobytes()
        
        response = stub.ProcessImage(image_service_pb2.ImageRequest(image_data=image_data))
        print(f'Result: {response.result}')

if __name__ == '__main__':
    run()
