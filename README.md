# Vertical-ZigZag

I have added a new mode in ArduCopter by modifying the zigzag mode. This mode makes the drone move in vertical zigzag pattern and takes input from Lidar/RangeFinder and does Wall distance correction when drone is moving against the wall. Wall distance correction works well in simulation but face many flaws when testing with real drone. Wall distance correction is based on waypoint navigation in which we store 2 points A and B, if drone moves from A to B then B is the next destination provided to WP_NAV controller. The wall correction distance is either subtracted or added in the next destination depending on the condition whether drone is away or near the wall.
The new mode file is mode_zigzagvert and you are welcome to contribute to improve the wall distance correction algorithn and meanwhile, I am still trying resolve issues with the actual flight tests.
