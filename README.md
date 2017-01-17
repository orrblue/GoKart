# Electric GoKart
Semi-Autonomous steering for Engineering Club Go Kart. 
This is a project that the Engineering Club at Foothill College was working on. They may have continued in a different repo.

The pictures included show some of the calculations and plans we made during the design stage.

When I worked on the project in 2016, we were using an Arduino Board but have since found that it is incapable of meeting our system's needs.

###Milestone 1
The first goal that my team and I worked on was to have the Go-Kart adjust its path and heading angle to be at a set distance and parallel to a wall.
It was planned that Go-Kart would be able to drive from any angle and any location until it got to be a predetermined distance away from a wall, and then drive parallel to it.
However, we ran into trouble when we found that the Arduino could not perform its tasks in a timely manner, due to a lack of multithreading (as far as we understood at the time).

It's intended tasks:
1. Calculate Go-Kart's heading angle using input from two ultrasonic proximity sensors
2. Drive steering motor (PMW) accurately and consistently to a desired angle.

Achievements: Calculate angle in relation to a wall using two proximity sensors. Identiy (with good certainty) a bottleneck in our system.

Changes I would make if I worked on this project again are:
1. Use a Raspberry Pi 3 or even a more capable computer.
2. Try to verify that my goal is achieveable with my hardware before trying to implement a solution.
3. Aim for a tougher milestone which would include the need for a camera and use photo recognition software such as OpenCV

