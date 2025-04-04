
import numpy as np
from naoqi import ALBroker, ALProxy
import cv2
import time
import math
import random
import threading

class ColorDetector:
    def __init__(self, IP="127.0.0.1", PORT=9559):
        # Picture RGB and HSV frames
        self.rgb_frame = None
        self.hsv_frame = None

        # object position
        self.object_x = None
        self.object_y = None
        self.object_area = None

        # target object color
        self.color_index = 0
        self.object_color = 'No color'

        self.myBroker = ALBroker("myBroker", "0.0.0.0", 0, IP, PORT)

        self.motion = ALProxy("ALMotion", IP, PORT)
        self.cameraProxy = ALProxy("ALVideoDevice", IP, PORT)
        self.tts = ALProxy("ALTextToSpeech", IP, PORT)
        self.posture = ALProxy("ALRobotPosture", IP, PORT)
        self.memory = ALProxy("ALMemory", IP, PORT)

        self.posture.goToPosture("StandInit", 1)

        # Start with the top camera
        self.camera_id = 0
        self.captureDevice = self.cameraProxy.subscribeCamera("python_client", self.camera_id, 2, 11, 15)

        self.turning_coef = 1.5
        self.is_walking = False
        self.stop_thread = False
        self.walking_thread = None
        self.attempts = 0

    def get_image(self):
        naoImage = self.cameraProxy.getImageRemote(self.captureDevice)
        if naoImage is None:
            print 'get_image: no image'
            return None

        width, height = naoImage[0], naoImage[1]
        array = naoImage[6]

        img = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
        if img is None:
            print "Failed to capture image"
            return

        self.rgb_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.hsv_frame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Function to switch between the top and bottom cameras in case missing "eye contact" with the object
    def switch_camera(self, camera_id):
        self.cameraProxy.unsubscribe(self.captureDevice)
        self.camera_id = camera_id
        self.captureDevice = self.cameraProxy.subscribeCamera("python_client", self.camera_id, 2, 11, 15)
        print "Switch to ", " top camera" if camera_id == 0 else "bottom camera"

    # Function to adjust brightness
    def adjust_gamma(self, image, gamma=1.2):
        inv_gamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv_gamma * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)

    # Function to get HSV limits for each color
    def get_limits_by_color(self, color_name):
        color_ranges = {
            "Red": [([0, 120, 100], [10, 255, 255]), ([167, 120, 100], [180, 255, 255])],
            "Orange": [([12, 100, 100], [24, 255, 255])],
            "Yellow": [([25, 100, 100], [35, 255, 255])],
            "Green": [([36, 100, 100], [85, 255, 255])],
            "Blue": [([86, 100, 100], [130, 255, 255])],
            "Violet": [([131, 100, 100], [166, 255, 255])],
        }
        return [(np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)) for lower, upper in
                color_ranges[color_name]]

    # Function to detect object position based on color
    def detect_object_position(self):
        if self.rgb_frame is None:
            return False

        limits = self.get_limits_by_color(self.object_color)
        mask = np.zeros(self.hsv_frame.shape[:2], dtype=np.uint8)
        for lowerLim, upperLim in limits:
            mask |= cv2.inRange(self.hsv_frame, lowerLim, upperLim)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if 150 < cv2.contourArea(c) < 10000]

        if not valid_contours:
            print "No object detected for color:", self.object_color
            self.attempts += 1

            # If we tried to check both of the cameras and didn't find anything, we will terminate
            if self.attempts >= 3:
                print "Object not found in both cameras. Terminating the program..."
                self.motion.stopMove()
                self.tts.say("I can't find the "+self.object_color+" anymore. Terminating.")

                if self.walking_thread and self.walking_thread.is_alive():
                    self.stop_thread = True
                    self.walking_thread.join()

                exit(0)

            # If we didn't found in top camera we will switch to bottom camera
            if self.camera_id == 0:
                print "Switching to bottom camera..."
                self.switch_camera(1) # Switching to bottom camera
            else:
                print "Switching to top camera..."
                self.switch_camera(0)

            time.sleep(1)
            self.get_image()
            return self.detect_object_position()

        self.attempts = 0

        target = min(valid_contours, key=lambda c: cv2.contourArea(c))
        x, y, w, h = cv2.boundingRect(target)
        self.object_area = w * h
        self.object_x = self.turning_coef * (x + w // 2 - mask.shape[1] // 2) / mask.shape[1]
        self.object_y = y * y / mask.shape[0]

        cv2.rectangle(self.rgb_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(self.rgb_frame, self.object_color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Function to turn the robot toward the detected object
    def turn_to_obj(self):
        if self.object_x is None:
            print "No object detected!\n"
            return False

        theta = self.object_x

        print "Turning to ", self.object_color, " angle: ", theta, " rad \n"

        if abs(theta) < 0.05:
            print "Already facing the object \n"
            return True

        self.motion.moveTo(0, 0, -theta)

        return True

    # Function to walk towards the object
    def walk_to_obj(self):
        self.is_walking = True
        self.motion.walkInit()
        previous_d = None

        while not self.stop_thread:
            if self.object_x is None:
                print "No object to follow \n"
                return False

            distance = self.object_y

            # Only print distance if it changes
            if previous_d != distance:
                print "Walking towards the object, distance: ", distance, "\n"
                previous_d = distance
                cv2.putText(robot.rgb_frame, "Walking towards the " + robot.object_color + " object",
                            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

            # Move toward if the distance is greater than 10 cm
            if distance > 218 or self.object_area < 11000:
                self.motion.moveToward(0.5, 0.0, 0) # Moving forward
            else:
                self.motion.stopMove()
                self.posture.goToPosture("StandInit", 1.0)
                print "Reached the ",self.object_color," object", "\n"
                self.tts.say("Reached the "+ self.object_color + " object")
                cv2.putText(self.rgb_frame, "Reached the " + self.object_color + " object",
                            (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                return True

        self.is_walking = False

# Create robot instance and start color tracking
robot = ColorDetector()

colors = ["Red", "Orange", "Yellow", "Green", "Blue", "Violet"]
robot.color_index = 0
robot.object_color = random.choice(colors)
print "Tracking color: " ,robot.object_color, "\n"

is_walking = False

def walk_thread(robot):
    robot.walk_to_obj()

while True:

    # Capturing the image
    robot.get_image()

    # Detection of object's placement
    robot.detect_object_position()

    if robot.rgb_frame is not None:
        cv2.imshow("NAO Camera", robot.rgb_frame)

    else:
        print 'No image'

    # Turn and walk towards the object if detected
    if robot.object_x is not None and robot.object_y is not None:
        if not robot.is_walking and (robot.walking_thread is None or not robot.walking_thread.is_alive()):
            turned = robot.turn_to_obj()
            if turned:
                print "starting to walk toward object..."
                robot.is_walking = True
                robot.stop_thread = False
                robot.walking_thread = threading.Thread(target=walk_thread, args=(robot,))
                robot.walking_thread.daemon = True
                robot.walking_thread.start()


        if robot.walking_thread and not robot.walking_thread.is_alive():
            print "Robot stopped. Terminating program..."
            break

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        robot.stop_thread = True
        if robot.walking_thread and robot.walking_thread.is_alive():
            robot.walking_thread.join()
        break

    time.sleep(1)


cv2.destroyAllWindows()