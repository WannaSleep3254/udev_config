# udev_config

## 시리얼 라이브러리
[https://github.com/wjwwood/serial.git](https://github.com/wjwwood/serial.git)

### CMakeLists.txt
```Bash
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)
```
### *.H
```C++
#ifndef IMU_NODE_H
#define IMU_NODE_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <sensor_msgs/Imu.h>

using namespace serial;

namespace Magenta
{

class IMU
{
public:
//  IMU(int argc, char** argv, ros::NodeHandle nh_);
  IMU(  int argc, char** argv, ros::NodeHandle nh_,
        const std::string &port, uint32_t baudrate,
        bytesize_t bytesize,     parity_t parity,
        stopbits_t stopbits
     );
    void setPort (const std::string &port);
    void setBaudrate (uint32_t baudrate);
    void setBytesize (bytesize_t bytesize);
    void setParity (parity_t parity);
    void setStopbits (stopbits_t stopbits);

    bool open();
    // run to 0xA1
    void read();
    void loop();

private:
    serial::Serial ser;
    ros::NodeHandle nh;
//  QTimer *loop_timer_;

  ros::Publisher publisher_imu_;
//  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscrber_cmd_;
  sensor_msgs::Imu imu_msgs;

  std::vector<uint8_t> buffer;
  size_t size;// = 0;
};

}

#endif
```

### *.CPP

## 파일경로
### Default rules
```Bash
cd /lib/udev/rules.d
```
### Temporary rules
``` Bash
cd /dev/.udev/rules.d
```
### user-cumstom ruels (*)
```Bash
cd /etc/udev/rules.d
```
## 작성 규칙
### 확장자
```Bash
*.rules
```
## 포트 권한설정
### sol-1) su 권한으로 실행
```Bash
sudo 
```

### sol-2) 포트 사용권한 허용
```Bash
sudo chmod 666 /dev/ttyACM0
```
### sol-3) 유저그룹 추가 [RECOMMAND]
```Bash
sudo usermod -a -G dialout $USER
```



## USB 포트 고정 or simulink 연결
 ### step-1) check VID & PID of device
```Bash
lsbusb
```
 ### step-2) check UID of device
```Bash
udevadm info -a -n /dev/ttyUSB* | grep '{serial}' | head -n1
```
 ### step-3) make RULE of simulink
1. 파일 생성

```Bash
cd /etc/udev/rules.d
sudo nano 99-usb-serial.rules
```
</br>
2. 포트입력

```Bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", ATTRS{serial}=="8573531333335161B142", SYMLINK+="arduino"
```
### step-4) rload rules
```Bash
udevadm control --reload-rules && udevadm trigger
```

### step-5) replug USB PORT & check result
 ```Bash
 ls -l /dev/arduino
 ```
