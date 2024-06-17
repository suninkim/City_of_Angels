import grpc
import sys
import os
from concurrent import futures
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import common.proto.image_service_pb2 as image_service_pb2
import common.proto.image_service_pb2_grpc as image_service_pb2_grpc



import cv2
import numpy as np
# import tensorflow as tf

class ImageServiceServicer(image_service_pb2_grpc.ImageServiceServicer):
    def __init__(self):
        a=1
        # EfficientDet 모델 로드 (예: EfficientDet-D0)
        # self.model = tf.saved_model.load('path/to/efficientdet')

    def ProcessImage(self, request, context):
        # 이미지 디코딩
        nparr = np.frombuffer(request.image_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        # 모델을 사용하여 이미지 처리
        # result = self.model(img)
        result = "a"
        
        # 결과를 문자열로 변환하여 응답
        return image_service_pb2.ImageResponse(result=str(result))

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    image_service_pb2_grpc.add_ImageServiceServicer_to_server(ImageServiceServicer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    server.wait_for_termination()

if __name__ == '__main__':
    serve()
