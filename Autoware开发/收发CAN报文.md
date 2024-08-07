## 收发CAN报文

#### Python：

```python
import can
```

创建CAN接口

```python
bus = can.interface.Bus(bustype='socketcan', channel='can1', bitrate=500000)
```

根据车辆协议书定义CAN报文读取ID

```py
read_ID = 0xFFF #FFF仅做示意
```

读取信号与发送信号框架

```python
while True:
    message = bus.recv() #读取CAN总线信息

    if message.arbitration_id == read_ID:#读取ID的数据
        data = (message.data[0] & 0x80) >> 7 #取第1组八位的第一位
        if data==1:
            print('第一位是1')  #执行相应操作，print仅做示意
        else:
            print('第一位是0')
        msg = can.Message(arbitration_id=read_ID, data=data, is_extended_id=False)
        bus.send(msg)
```

