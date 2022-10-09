import imutils
frame = cv2.imread('output/final.png')
orig_image = cv2.imread('test_orig.png')
H, W = orig_image.shape[:2]
height, width = frame.shape[:2]

line_note_mask = cv2.inRange(frame, (0,0,0), (0, 0, 0))
gap_note_mask = cv2.inRange(frame, (255, 255, 255), (255, 255, 255))


filters = [("line_note", np.array([0, 0, 0]), np.array([0, 0, 0])), \
            ("gap_note", np.array([255, 255, 255]), np.array([255, 255, 255]))]

boxes = {}

for name, lower_bound, upper_bound in filters:
    mask = cv2.inRange(frame, lower_bound, upper_bound)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    if len(cnts)>0:
        curr_boxes = []
        for (i, c) in enumerate(cnts):
            print(cv2.contourArea(c))
            if cv2.contourArea(c) < 240:
                continue
            box = cv2.minAreaRect(c)
            box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
            box = np.array(box, dtype="int")
            cv2.drawContours(orig_image, [box], -1, (0, 0, 255), 2)
            
            rect = order_points(box)
            center = ((rect[0][1] + rect[1][1] + rect[2][1] + rect[3][1])/4, (rect[0][0] + rect[1][0] + rect[2][0] + rect[3][0])/4)
            colors = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))
            
            # Get properly oriented bounding box
            # x, y, w, h = get_width_height(rect, height, width)
            
            x, y, w, h = get_width_height(rect, H, W)
            center = (x + int(w/2), y + int(h/2))
            curr_boxes.append(center)
            
            ##Place the note
            _, note, _, minDist = find_note(center[1], name)
            
            cv2.circle(orig_image, (int(center[0]), int(center[1])) , 5, (0, 255, 0), -1)
            
#             for ((x, y), color) in zip(rect, colors):
#                 cv2.circle(orig_image, (int(x), int(y)), 5, color, -1)

            cv2.putText(orig_image, "{}={}".format(minDist,note),
                (int(rect[0][0] - 5), int(rect[0][1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
        cv2.imwrite('output/annotted.png', orig_image)
        boxes[name] = curr_boxes
#print(boxes)