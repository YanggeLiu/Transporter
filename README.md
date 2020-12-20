# Transporter
A system which group transportation

## **硬件材料**
- 树莓派3B 或以上型号
- Arduino
- (可遥控越野车)车架及其配件

## **功能**
- 可识别障碍物并沿通道运行
-  拟增加PID算法

## **使用前**
- download opencv(4.2.0) opencv_contrib(4.2.0)
- 此脚本仅适用于树莓派
```
$ cd /path/auto_build_opencv

$ wget -c https://github.com/opencv/opencv/archive/4.2.0.zip -O opencv-4.2.0.zip

$ wget -c https://github.com/opencv/opencv_contrib/archive/4.2.0 -O opencv_contrib-4.2.0.zip
```

## **使用**
```
bash run.sh
```
