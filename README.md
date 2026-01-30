## 0126备忘

### 1.模式切换

    mavlink中的协议中的customMode id和此id不同,所以解析出的模式名称仅供参考

### 2.仿真方法 仿真飞控起飞点坐标(lon:-122.3895140,lat:37.62785727)

### 3.整体流程跑通

### 4.回传实时坐标(心跳包中)

### 5.当前为仿真模式,切换回正常模式的步骤

    command_ACK
    find_modes
    direct_fc
    mission_planner

## 代码逻辑

### ground_station.py   
### Windows与Linux间局域网通信

    包含下行视频流,下行实时坐标,上行路径

### mission_planner.py  
### Windows(DEM数据读取,路径规划)
### Linux(input路径,output飞控指令)

### direct_fc.py
### V12测试成功



## 降落后关闭电机指令
## 直线飞行

## 退回正常固件

## FT232 芯片接线逻辑
    红线-VCC
    黑线-GND->GND
    白线-RXD->TX
    绿线-TXD->RX

