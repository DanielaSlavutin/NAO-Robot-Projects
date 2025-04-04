# NAO-Robot-Projects
Welcome to the NAO Robot Projects repository! This reposiroty contains various projects focused on programming the NAO humanoid robot, particularly in the field of computer vision and interactive feedback.

## 🚀 Projects
### 1️⃣ Color Detection with Voice Feedback ✅
   A project where the NAO robot detects colors in its field of view and provides voice feedback on the detected color's position.

### 2️⃣ Color Detection with Motoric Feedback in Webots✅
   A project where the NAO robot detects colors in its field of view and walks towards the colored object once detected.

## 🛠 Setup & Requirements
To run these projects, you will need:
* A NAO robot or a simulation environment such as Webots (I recommend to use specifically Webots R2019a)
* Python 2.7 (due to NAO's SDK limitations)
* NAOqi SDK installed
* OpenCV and Numpy libraries

## 🏃‍♂️ How to Use

### Running on a Physical NAO Robot
1. Clone the repository
2. Navigate to the desired project folder
3. Connect your PC to NAO's IP address
4. Open the Script "Color_Detector_Talk.py" or "Color_Detector.py"
5. Change to your robot's IP address in this section in the script:
def __init__(self, IP="127.0.0.1", PORT=9559):
6. Run the script

### Running in Webots Simulator
1. Install Webots and configure it for NAO simulation
2. Open Webots and load the NAO robot environement
3. Clone the repository and navigate to the project folder
4. Navigate to where the NAOqi is loacted in C:\
5. Open CMD or PowerShell in there and use the command "naoqi.py -w" or ".\naoqi.py -w"
6. Open the Script "Color_Detector_Talk.py" or "Color_Detector.py"
