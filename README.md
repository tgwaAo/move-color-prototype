# Move Color Prototype
This is a little game. In this game, you move a bright color (must not be bright,
but it is better for fast movements) through the window, try to hit the bright
green circles and try to avoid bright red circles. The dark circles are not ready and become active after a short time. You have one minute.

Requirements:
Requires OpenCV

https://opencv.org/

for camera usage and eigen 

https://eigen.tuxfamily.org/index.php?title=Main_Page

for calibration.

Gamestart:
The game will start a timer at first start and makes a temporary picture. This 
picture will be used to calibrate. A calibration is nesseccary before first 
gameplay.

Controls:
r = replay
c = calibrate
d = debug
esc = end (gameplay and result)

while calibration:
r = select rectangles again
enter = accept
esc = abort

PLEASE BE CAREFUL WHILE PLAYING AND MAKE SURE TO READ THE LICENSE.
