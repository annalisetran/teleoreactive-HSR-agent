from ultralytics import YOLO
'''
This is a template script to train a YOLO model on a custom dataset using the Ultralytics library.
This script is used for learning from scratch, where the model is trained from scratch on the custom dataset.
Use this as a template to train your own YOLO model to detect custom objects.
'''
# Load a model

model = YOLO("yolov8n.pt")  # build a new model form scratch

# Use the model
#model.train(data="coco8.yaml", epochs=3)
model.train(data="/home/blinky/roboworkspace/src/VISION/ultralytic_training_pipeline/train_transfer.yaml", epochs=3)  # train the model
metrics = model.val()  # evaluate model performance on the validation set

