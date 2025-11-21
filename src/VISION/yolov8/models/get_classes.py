import ultralytics

model=ultralytics.YOLO("generalModel_best.pt")

print(model.names)

