<launch>

  <!-- Start the segwayrmp SmartCar node -->
  <node
    pkg="segwayrmp"
    type="SmartCar"
    name="SmartCar"
    output="screen"/>

  <!-- Start the joy_teleop node -->
  <node
    pkg="joy"
    type="joy_teleop"
    name="joy_teleop"
    output="screen"/>

  <!-- Start the rmp220_teleop.py node -->
  <node
    pkg="rmp220_teleop"
    type="rmp220_teleop.py"
    name="rmp220_teleop"
    output="screen"/>

</launch>
