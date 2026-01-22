# Notes

## Links

- [Examples repo](https://github.com/MISTI-GTL-2026-CY/examples)
- [Day 2 presentation (links to cheatsheets)](https://docs.google.com/presentation/d/19lr2Xdv0ODMIGynH387BWq3_1VS4X9pad5Smx370zwc/edit?slide=id.g3b56e9850b2_17_201#slide=id.g3b56e9850b2_17_201)

## Running

### From the device where the changes were made
In the `ProjectT` folder
1. `git add .`
2. `git commit -m "m"`
3. `git push`

**Note**: To check at any time use `git status`

---

### SSH (First time)
Connect to wifi 1

`ssh duckie@duckie05.local`

---

### To run the robot
Make sure to exit the container between runs!

In the `projectt/projectt` folder:
1. `git pull` (Should show some changes made)
2. `make build`
3. `make run`
4. In the container: `bash run.sh`

`run.sh` should contain:
```bash
source /opt/ros/humble/setup.bash 
source ./install/setup.bash 
export VEHICLE_NAME=duckie05
export USER_NAME=duckie05
chmod a+x ./install/blank_package/lib/blank_package/blank_node.py
ls -al ./install/blank_package/lib/blank_package/
ros2 run blank_package blank_node.py
```

---

## ROS

### Main template
```py
class [NAME](Node):
    def __init__(self):
        super().__init__('[NAME LOWERCASE]')
        self.vehicle_name = os.getenv('VEHICLE_NAME')

def main():
    rclpy.init()
    blinker = Blinker()
    rclpy.spin(blinker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### LEDs
Message type: `LEDPattern`  
Topic: `/duckie05/led_pattern`

Libraries:
```py
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
```

In `__init()__`:

```py
self.vehicle_name = os.getenv('VEHICLE_NAME')
self.led_pub = self.create_publisher(LEDPattern, f'/{self.vehicle_name}/led_pattern', 1)
```

Template to publish:
```py
pattern = ColorRGBA(r=[RED], g=[GREEN], b=[BLUE], a=1.0)
msg.rgb_vals = [pattern] * 5
self.led_pub.publish(msg)
```

---

### Wheels
Message type: `LEDPattern`  
Topic: `/duckie05/wheels_cmd`

Libraries:
```py
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header
```

In `__init()__`:

```py
self.vehicle_name = os.getenv('VEHICLE_NAME')
self.wheel_pub = self.create_publisher(WheelsCmdStamped, f'/{self.vehicle_name}/wheels_cmd', 1)
```

Template to publish:
```py
wheel_msg = WheelsCmdStamped()

# Do we actually need these?
header = Header()
header.stamp = self.get_clock().now().to_msg()

wheel_msg.header = header
wheel_msg.vel_left = [LEFT VELOCITY]
wheel_msg.vel_right = [RIGHT VELOCITY]

self.wheel_pub.publish(wheel_msg)
```

---

### Camera

[TODO] Finish + check these tomorrow ig

Message type: `CompressedImage`


Libraries:
```py
from sensor_msgs.msg import CompressedImage
```

In `__init()__`:
```py
self.output_dir = "/workspace/images/"
os.makedirs(self.output_dir, exist_ok=True)
self.vehicle_name = os.getenv('VEHICLE_NAME')
self.counter = 0
self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)
```

To save the images:
```py
if self.counter % 30 == 0:
    with open(self.output_dir + str(self.counter) + '.jpg', 'wb') as f:
        self.get_logger().info(f'Saving image {self.counter}')
        f.write(msg.data)
self.counter += 1
```

---

## Troubleshooting
If Error 123/125 etc. show up make sure:
- `requirements-apt.txt` is empty
- All `.` in `Makefile` are replaced by `$(CUR_DIR)`\
Should look like:
```make
CUR_DIR = $(shell pwd)

build:
	docker build -t "$$(basename $(CUR_DIR))-duckiebot-image" --progress=plain .
	docker run --rm -v $(CUR_DIR)/:/workspace --entrypoint=colcon "$$(basename $(CUR_DIR))-duckiebot-image" build

run:
	docker run --rm -it --network=host --privileged -v /dev/shm:/dev/shm -v $(CUR_DIR)/:/workspace "$$(basename $(CUR_DIR))-duckiebot-image"

.PHONY: run build
```