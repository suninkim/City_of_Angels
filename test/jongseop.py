import streamlit as st
from PIL import Image
from ultralytics import YOLO

# Load a model
# model = YOLO("yolov8n.yaml")  # build a new model from scratch
model = YOLO("yolov8n-seg.pt")

# Use the model
# model.train(data="coco8.yaml", epochs=3)  # train the model
# metrics = model.val()  # evaluate model performance on the validation set
results = model("vision/test.png")  # predict on an image
import cv2

# plot()
# verbos()
# save_crop(save_dir=".")
print(results[0].summary()[0]["box"])
cv2.imwrite(
    "asd.png",
    results[0].plot(),
)
