from ultralytics import YOLO
model = YOLO('best.pt')  # модель
results = model('input.jpg')
result = results[0]
result.show()
#image = result.plot()
boxes = result.boxes
xywh = boxes.xywh.tolist()
if len(xywh) != 0:
  print("DETECTED")
  print(xywh)

