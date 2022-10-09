import cv2

capture = cv2.VideoCapture('./JustMast.mp4')

frameNr = 0
 
while (True):
 
    success, frame = capture.read()
 
    if success:
        cv2.imwrite(f'./mast_imgs/frame_{frameNr}.jpg', frame)
 
    else:
        break
 
    frameNr = frameNr+1
 
capture.release()
