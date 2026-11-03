# Summary
This software is a human interface for a small agricultral robot. 
The robot is intended for small properties (in the range of 1 to 5 acres).
It has three capabilities:
* Area Spray. Cover the entire designated area.
* Spot Spray. Detect weeds and spray them individually.
* Mow. Mow entire designated area.

# Use Environemnt
This softare is to run on laptops, phones and tablets. It is intennded mainly to be used in an office/controled environment via remote link. Users will be tech litterate, but not engineers.

# Pages
A wireframe diagram is included in `resources`.
The dashboard should have a menu on the side with options for field managment, spraying, mowing and maintenance. Only one page should be open at a time.

## Field Managment
This page allows the user to add, edit and view fields. Fields are polygons with designated headlands and known obstacles or keep-out zones. They must be saved in non-volatile storage.
New fields shall be added either by drawing a polygon on a map, or by driving the robot manually around the desired area while it records RTK GNSS data.

## Area Spray
This will communicate recessary data to the Field2Cover action server allowing a the robot to begin its activity. It shall allow the user to enter data such as litres per sq meter, tank capacity, boom width.

## Spot Spray
Leave blank

## Mow
Leave blank

## Robot
This page shall allow the user to view the time the robot has been operating, how much distance it has covered and when the next maintenance should be. It shall allow the user to view the temeprature of the robot, alerts, battery percentage, link strength, current action, and firmware version. 
This page should provide ability to test motors and view sensor information.

# Robot Interface
the robot runs with ROS2. It uses Nav2 along with Field2Cover for generating field coverage routes. The robot has a long-range WiFi link allowing constant connection from anywhere on the property, however it can run without a connection as well.

# Starting Point
Consider using QGIS.
A web application is preferable. 
Code should be written with the intent to run on Android and iPhone in the future, but for now laptop only needs to be supported.