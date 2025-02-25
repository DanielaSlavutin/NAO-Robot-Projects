import numpy as np
from naoqi import ALBroker, ALProxy
import cv2
import time

class ColorDetector:
    def __init__(self, IP="127.0.0.1", PORT=9559):
        # Picture RGB and HSV frames
        self.rgb_frame = None
        self.hsv_frame = None

        # Object position - X and Y coordinates
        self.object_x = None
        self.object_y = None

        # Target colored object
        self.color_index = 0
        self.object_color = 'No color'

        # Flag parameter to print the color's direction only once
        self.message_shown = False

        self.myBroker = ALBroker("myBroker", "0.0.0.0", 0, IP, PORT)

        self.motion = ALProxy("ALMotion", IP, PORT)
        self.cameraProxy = ALProxy("ALVideoDevice", IP, PORT)
        self.tts = ALProxy("ALTextToSpeech", IP, PORT)
        self.posture = ALProxy("ALRobotPosture", IP, PORT)

        self.posture.goToPosture("StandInit", 0.5)

        self.camera_id =0
        self.captureDevice = self.cameraProxy.subscribeCamera("python_client", self.camera_id, 2, 11, 15)

    def get_image(self):
        naoImage = self.cameraProxy.getImageRemote(self.captureDevice)
        if naoImage is None:
            return None

        width, height = naoImage[0], naoImage[1]
        array = naoImage[6]

        img = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
        if img is None:
            print "Failed to capture image"
            return

        self.rgb_frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.hsv_frame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


    # Function to adjust brightness
    def adjust_gamma(selfself, image, gamma = 1.2):
        inv_gamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv_gamma * 255 for i in range(256)]).astype("uint8")
        return cv2.LUT(image, table)

    # Function to get HSV limits for each colors
    def get_limits_by_color(self, color_name):
        color_ranges = {
            "Red": [([0, 100, 100], [10, 255, 255]), ([170, 100, 100], [180, 255, 255])],
            "Orange": [([11, 100, 100], [22, 255, 255])],
            "Yellow": [([23, 100, 100], [33, 255, 255])],
            "Green": [([34, 100, 100], [78, 255, 255])],
            "Blue": [([79, 100, 100], [131, 255, 255])],
            "Violet": [([132, 100, 100], [170, 255, 255])],
        }
        return [(np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)) for lower, upper in
                color_ranges[color_name]]

    # Function to detect ojbect position based on color
    def detect_colored_object_position(self):
        if self.rgb_frame is None:
            return None

        limits = self.get_limits_by_color(self.object_color)
        mask = np.zeros(self.hsv_frame.shape[:2], dtype=np.uint8)
        for lowerLim, upperLim in limits:
            mask |= cv2.inRange(self.hsv_frame, lowerLim, upperLim)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if 50 < cv2.contourArea(c) < 5000]

        if not valid_contours:
            print "No object detected for color:", self.object_color
            return None

        target = min(valid_contours, key=lambda c: cv2.contourArea(c))
        x, y, w, h = cv2.boundingRect(target)
        self.object_x = 1.5 * (x + w // 2 - mask.shape[1] // 2) / mask.shape[1]
        self.object_y = 2. * (y + h // 2 - mask.shape[0] // 2) / mask.shape[0]

        cv2.rectangle(self.rgb_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.putText(self.rgb_frame, self.object_color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # Function to tell where the colored object is
    def tell_object_position(self):
        self.detect_colored_object_position()

        if self.object_x is None and self.object_y is None:
            self.tts.say("I can't find any " + self.object_color + " object.")
            return None

        direction_x = "left" if self.object_x < 0 else "right"
        direction_y = "higher" if self.object_y > 0 else "lower"

        if not self.message_shown:
            print self.object_color + " object is " + direction_x + ", " + direction_y
            self.tts.say("The " + self.object_color + " object is on the " + direction_x + " side and " + direction_y + " in the frame.")
            self.message_shown = True

        return self.rgb_frame


robot = ColorDetector()

colors = ["Red", "Orange", "Yellow", "Green", "Blue", "Violet"]
robot.color_index = 0
robot.object_color = colors[robot.color_index]

while True:
    robot.get_image()
    robot.tell_object_position()

    if robot.rgb_frame is not None:
        cv2.imshow("NAO Camera", robot.rgb_frame)
    else:
        print 'No Image'

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('n'):
        robot.color_index = (robot.color_index + 1) % len(colors)
        robot.object_color = colors[robot.color_index]
        robot.tts.say("Now tracking " + robot.object_color)
        print("Switched to color: " + robot.object_color)
        time.sleep(1)
        robot.message_shown = False

cv2.destroyAllWindows()

