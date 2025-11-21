from ultralytics import YOLO
'''
This is a template script to train a YOLO model on a custom dataset using the Ultralytics library.
This script is used for transfer learning, where the model is pretrained on the COCO dataset and then fine-tuned on the custom dataset.
Use this as a template to train your own YOLO model based on an existing model.
'''
# Load a model

model = YOLO("yolov8n.yaml")  # load a pretrained model (recommended for training)

# Use the model
#model.train(data="coco8.yaml", epochs=3)
model.train(data="/home/blinky/roboworkspace/src/VISION/ultralytic_training_pipeline/train_transfer.yaml", epochs=3, pretrained=True)  # train the model
metrics = model.val()  # evaluate model performance on the validation set

