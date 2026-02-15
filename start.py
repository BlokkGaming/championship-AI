from ultralytics import YOLO
model = YOLO('best.pt')  # модель
results = model('input.png', verbose = False)
results[0].show()
boxes = results[0].boxes
xywh = boxes.xywh.tolist()
print(xywh)

