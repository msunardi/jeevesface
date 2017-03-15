# Jeeves Face Animation

Adapted from [AdaFruit](https://learn.adafruit.com/animating-multiple-led-backpacks)

**jeevesface_ros_3** supports ROS (rosserial [learn](http://wiki.ros.org/rosserial_arduino/Tutorials/Blink))

-  Subscribes to topic: `/face` (type: `std_msgs/String`)
-  Publishes to topic: `/chatter` (type: `std_msgs/String`)

### To run:

1. Upload `jeevesface_ros_3.ino` to an Arduino. **NOTE:** Requires Arduino MEGA2560 or similar with SRAM > 3kB and Flash > 32kB. Tested compile/upload using Arduino IDE v1.8.1. On MEGA2560, SDA -> pin 20, SCL -> pin 21.
2. Run `rosserial`:

    ```
    rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
    ```
3. To test, publish to `/face` topic.
  - For mouth shapes:
    - Input characters:
      - `'i'`: closed
      - `'o'`,`'p'`,`'j'`: slightly open to wide smile
      - `'k'`,`'l'`: 'u' and 'o' shapes
      - `'f'`: frown
      - `'g'` or other characters: normal/random
    - Sample usage - publish topic from command line:

        <code>
        rostopic pub /face std_msgs/String "{data: 'p'}" --once

        OR

        rostopic pub /face std_msgs/String p --once
        </code>

  - For eye shapes:
    - Input characters:
      - `'q'`,`'r'`: eyes fully open
      - `'w'`: eyes slightly closed
      - `'e'`: eyes fully closed (one pixel wide)
    - Sample usage - same as above
  - For eye positions:
    - Format: `'x:y'`
      - `x`: integer [1-5] - horizontal position of pupil
      - `y`: integer [1-5] - vertical position of pupil
    - Sample usage - publish topic from command line:

        <code>
        rostopic pub /face std_msgs/String "{data: '1:3'}" --once

        rostopic pub /face std_msgs/String "{data: '4:2'}" --once
        </code>
