cornTracker is a simple ROS 2 node that helps a robot detect corn in agriculture fields. It watches the robot's camera feed, listens to its GPS location, and checks for 3D object detections coming from a YOLO model.

Whenever it sees a corn with good enough confidence, it figures out the real GPS location of that corn based on the robot’s current position. Then it takes a cropped image of the corn from the camera and saves it in a folder called images.

It also logs all the important info into a CSV file — things like the corn’s ID, its GPS coordinates, the image filename, and the detection confidence. If it already saw that corn before, it skips it, so everything in the file is unique.

This node is useful if you want to map corn locations while a robot drives around the field. You end up with a folder of pictures and a nice spreadsheet with the location of every corn it found.