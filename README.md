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
  IMU(  int argc, char** argv, ros::NodeHandle nh_,
        const std::string &port, uint32_t baudrate,
        bytesize_t bytesize,     parity_t parity,
        stopbits_t stopbits
     );
     
    void read();
    
private:
    serial::Serial ser;
    ros::NodeHandle nh;

    std::vector<uint8_t> buffer;
    size_t size;
};

}

#endif
```

### *.CPP
```C++
IMU::IMU(    int argc, char** argv, ros::NodeHandle nh_,
            const std::string &port, uint32_t baudrate,
            bytesize_t bytesize,     parity_t parity,
            stopbits_t stopbits)
{
   ros::init(argc, argv, "serial_control");
    nh = nh_;
    ser.setPort("/dev/imu");
    ser.setBaudrate(115200);
    ser.setBytesize(serial::eightbits);
    ser.setParity(serial::parity_none);
    ser.setStopbits(serial::stopbits_one);

    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
}

void IMU::read()
{
    size = 0;
    size = ser.available();
    buffer.clear();

    if(size == 0)
    {
        return;
    }
    std::string read= ser.readline();

    std::vector<std::string> words;
    std::stringstream sstream(read);
    std::istringstream isstream(read);
    std::string word;

    while(isstream>>word)
    {
        words.push_back(word);
    }

    if(words.size()!=9)
        return;
/*
    for(int i=0; i<words.size(); i++)
    {
        std::cout<< std::stod(words[i] );
    }
    std::cout<<std::endl;
*/
}
```

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
