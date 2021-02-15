
import cv2
import numpy as np


print(cv2.__version__)
# Load Yolo
net = cv2.dnn.readNet("/home/adamanov/fav/catkin_ws/src/face_detection/nodes/dnn/yolov3.weights", "/home/adamanov/fav/catkin_ws/src/face_detection/nodes/dnn/yolov3.cfg")
classes = []
with open("/home/adamanov/fav/catkin_ws/src/face_detection/nodes/dnn/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3))



#feed with webcamimg
cam = cv2.VideoCapture(0)

if cam.read() == False: 
     cam.open()
if not cam.isOpened():
    print("cant open camera")
    exit()

while(True):
    # Capture frame-by-frame
    ret, img = cam.read()

    # Our operations on the frame come here
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.resize(img, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape
    image_center = [width/2, height/2]
    blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    # Showing informations on the screen
    class_ids = []
    confidences = []
    boxes = []
    center = []
    persons_center = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                cv2.circle(img, (center_x, center_y), 8, (0, 255, 0), 2)

                print(center_x, center_y)
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                center.append([center_x, center_y])
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    font = cv2.FONT_HERSHEY_PLAIN
    p = 0
    for i in range(len(boxes)):
        if i in indexes:
            
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            print(label, i, center[i])
            color = colors[class_ids[i]]
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
            if class_ids[i] == 0: 
                persons_center.append(center[i])
                p = p + 1

    print('Amount of persons seen:', p)
    print('Center of persons:',persons_center)
    print('Center of image:',image_center)

    # Display the resulting frame
    cv2.imshow('img',img)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cam.release()    
cv2.destroyAllWindows()

