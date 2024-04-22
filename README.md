# OLEI LiDAR

IPv4 Manual:
* Address: 192.168.1.99
* Netmask: 255.255.255.0

1. Build
``` sh
cd ~/catkin_ws && catkin_make
```
2. Clone `test_ws`
```sh
git clone https://github.com/DucTuan7isme/LiDAR-Project.git
```
3. Repeat step 1
``` sh
cd ~/catkin_ws && catkin_make
```


## Usage 

* In the first Terminal, run: (Trong Terminal đầu tiên, chạy:)
```sh
roscore
```
* To start the scan of the LiDAR, open another Terminal and Run: (Để bắt đầu quét LiDAR, mở một Terminal khác và chạy:)
```sh
roslaunch olelidar scan.launch
```
* To observe the scan in RViz, in other Terminal, open Rviz and Run file `test.rviz` in folder `rviz`: (Để quan sát quá trình quét trong RViz, trong Terminal khác, mở Rviz và chạy file `test.rviz` trong thư mục `rviz`)

* To denoise the raw LaserScan of the LiDAR, open another Terminal and run: (Để khử nhiễu dữ liệu thô của LiDAR, mở một Terminal khác và chạy:)
```sh
rosrun olelidar denoise
```
To see the LaserScan after denoising, in Rviz, change the topic from "front_scan" in LaserScan to "denoise_node". (Để xem tia quét cuar LiDAR sau khi khử nhiễu, trong Rviz, hãy thay đổi chủ đề từ "front_scan" trong LaserScan thành "denoise_node".)

* To measure the distance between the LiDAR and the milestone and distinguish it with other materials, open another Terminal and run: (Để đo khoảng cách giữa LiDAR và gương tròn cũng như phân biệt nó với các vật liệu khác, hãy mở một Terminal khác và chạy:)
```sh
rosrun olelidar circle_regression
```
Or
```sh
rosrun olelidar shape_detection
```




