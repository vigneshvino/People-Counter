# People-Counter
This project is to count people going IN and OUT using opencv library. It was done on raspberry pi along with picamera module installed on it.

Follow the steps to deploy the code on raspberry pi.

1. Installation of opencv
   Execute the following commands on terminal.
   ```
   sudo apt-get update
   sudo apt-get upgrade
   sudo apt-get install build-essential 
   sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev 
   sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev  libjpeg-dev   libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev 
   sudo apt-get install python-opencv 
   sudo apt-get install python-matplotlib
   ```
   verify the OpenCV install
   ```
   python
   >>> import cv2
   >>> cv2.__version__        # will say "2.4.9" if pre-compiled binary installation was successful
   ```
2. To enable the camera refer this [link](https://www.hackster.io/deligence-technologies/person-counting-system-using-opencv-and-python-faf14f).

3.	Execute the Detector.py file
Place the `detector.py` file on home location and execute that with the following command.
`python detector.py`
