# OLEI LiDAR

I. Build
``` sh
cd ~/catkin_ws && catkin_make
```
II. Clone `test_ws`
```sh
git clone https://github.com/DucTuan7isme/LiDAR-Project.git
```
III. Repeat step I
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
* To measure the distance between the LiDAR and the milestone and distinguish it with other materials, open another Terminal and run: (Để đo khoảng cách giữa LiDAR và gương tròn cũng như phân biệt nó với các vật liệu khác, hãy mở một Terminal khác và chạy:)
```sh
rosrun olelidar circle_regression
```
Or
```sh
rosrun olelidar shape_detection
```




