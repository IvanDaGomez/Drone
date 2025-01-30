import sys
import os
# from drone.sensors.MPU650 import MPU650
# Add the cvModules folder to Python's module search path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'cvModules')))

from drone import Drone
import time 
import cv2
#from handDetector import HandDetector
drone = Drone(servo_port = 18, 
    motor1_port = 22, enablePin1_port = 27
    )
#detector = HandDetector()
def main():
    Ptime = 0
    count = 0
    lastFps = 0
    
    while True:
        rgb_frame = drone.camera.capture_array()
        bgr_img = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        #bgr_img = detector.findHands(bgr_img)    
        # Show fps
        drone.servo.angle = count
        
        Ctime = time.time()
        fps = 1 / (Ctime - Ptime)
        Ptime = Ctime
        count += 1
        # print(count)
        cv2.putText(bgr_img, f'FPS: {str(int(lastFps))}', (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        if count > 10:
            lastFps = fps
            count = 0
        drone.turn_motors_on(fps)
        # Show the frame using OpenCV
        cv2.imshow("Pi Camera Preview", bgr_img)

        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    main()
