import os
import sys
from concurrent import futures

import grpc
from torchvision import transforms

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))


import cv2
import numpy as np
from ultralytics import YOLO

import common.proto.image_service_pb2 as image_service_pb2
import common.proto.image_service_pb2_grpc as image_service_pb2_grpc

# import tensorflow as tf


class ImageServiceServicer(image_service_pb2_grpc.ImageServiceServicer):
    def __init__(self):
        self.model = YOLO("vision/weights/yolov8n-seg.pt")
        self.tfms = transforms.Compose(
            [
                transforms.Resize(224),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )

    def ProcessImage(self, request, context):

        nparr = np.frombuffer(request.image_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        result = self.model(img)
        detected_img = result[0].plot()
        detected_result = result[0].summary()

        return image_service_pb2.ImageResponse(result=result[0].summary()[0]["name"])


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    image_service_pb2_grpc.add_ImageServiceServicer_to_server(
        ImageServiceServicer(), server
    )
    server.add_insecure_port("[::]:50051")
    server.start()
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
