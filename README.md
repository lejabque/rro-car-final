## Computer Vision system for autonomus car <br>(Russian Robotics Olympiad 2018)
### Python3 + OpenCV3

Self-driving car model
<br>
<img src="https://habrastorage.org/webt/3l/sd/x7/3lsdx7wiyhgmnoff5unxqxcqsrw.jpeg" width="420" height="400" border="10"/>
<img src="https://habrastorage.org/webt/hv/jf/xn/hvjfxnredlzqn_w0lqinb-naj4w.jpeg" width="420" height="400" border="10"/> 
<br>
This project builds a self-driving car using Raspberry Pi, Arduino and open source software.
<br>
Raspberry Pi collects inputs from a camera, processes input images for object detection (road signs, line and traffic light).
<br>
Data is then sent to the Arduino for car control. 
Example of line detection:
<img src="line.gif" width="640" height="480" />
<br>


### About the files
**straming/**  
  &emsp; &emsp; `client_cv.py`: video streaming from Pi to computer  
  &emsp; &emsp;  `server_cv.py`: video receiving on computer
  
**tasks/**  
  &emsp; &emsp;  `1_car-test.py`: Task 1: CV system test
  <br>
  &emsp; &emsp;  `2_circular.py`: Task 2: Travel of the robot car along a circular trajectory
  <br>
  &emsp; &emsp;  `3_U.py`: Task 3: Travel of the robot car along a 'U' trajectory
  <br>
  &emsp; &emsp;  `4_ext-circle.py`: Task 4: Travel of the robot car along an external circle
  <br>
  &emsp; &emsp;  `5_known-route.py`: Task 5: Travel of the robot car along a known ruute
  <br>
  &emsp; &emsp;  `6_unknown-route.py`: Task 6: Travel of the robot car along an unknown route 
  <br>
  
**sign/**    
  &emsp; &emsp;  **haar/**  
      &emsp; &emsp;  &emsp; &emsp;  trained HAAR cascade classifiers  
  &emsp; &emsp;  **lbp/**   
      &emsp; &emsp;  &emsp; &emsp;  trained LBP cascade classifiers  
      
`save.py`:     Video capture script
<br>
`RRO2018_AIRS_9-11kl_rules_v2.3.pdf`: Olympiad rules
