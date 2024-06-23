import os
import sys

import cv2
import grpc

# 프로젝트 루트 경로를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import common.proto.image_service_pb2 as image_service_pb2
import common.proto.image_service_pb2_grpc as image_service_pb2_grpc


def capture_image_from_arducam():
    # Arducam으로부터 이미지 캡처 (예시)
    cap = cv2.VideoCapture(1, cv2.CAP_V4L2)  # 카메라 ID를 0으로 설정
    cap.set(cv2.CAP_PROP_FPS, 20)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ret, frame = cap.read()
    # cap.release()
    return frame


def run():
    with grpc.insecure_channel("localhost:50051") as channel:
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

            _, img_encoded = cv2.imencode(".jpg", image)
            image_data = img_encoded.tobytes()

            response = stub.ProcessImage(
                image_service_pb2.ImageRequest(image_data=image_data)
            )
            print(f"Result: {response.result}")

        cv2.destroyAllWindows()


def send_image(image_path):
    """
    이미지 파일을 읽어서 gRPC를 통해 서버로 전송하는 함수.
    :param image_path: 이미지 파일의 경로
    """
    with grpc.insecure_channel("localhost:50051") as channel:
        stub = image_service_pb2_grpc.ImageServiceStub(channel)

        # 이미지 읽기
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        # cv2.imshow("dsd", image)
        # cv2.waitKey(0)
        if image is None:
            print(f"Error: Unable to read image from {image_path}")
            return

        # 이미지 인코딩
        _, img_encoded = cv2.imencode(".jpg", image)
        image_data = img_encoded.tobytes()

        # gRPC 요청 생성 및 전송
        request = image_service_pb2.ImageRequest(image_data=image_data)
        response = stub.ProcessImage(request)

        print(f"Result: {response.result}")


if __name__ == "__main__":
    # run()
    send_image("vision/test.png")
