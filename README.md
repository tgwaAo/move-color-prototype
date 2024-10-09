# Move Color Prototype

This is a little prototype of a game. 

## DISCLAIMER

THIS GAME DOES NOT CHECK YOUR SURROUNDINGS. PLEASE BE CAREFUL WHILE PLAYING.

## Camera choice

This program would normally use the first camera at position 0. If you want to use another camera, create a cam.txt with the necessary number in the directory of the executable.  
You can find the number in the following way:  

### Windows
* TODO

### Linux
* Get a list of devices with `ls /dev/vid*`.
* Choose only even numbers. E.g. the second camera is 2.

### Mac

* TODO

## Before gameplay
If you have not configured the game yet, the calibration will start automatically.

## Gameplay
In this game, you move a color through the window, try to hit the bright green circles and try to avoid bright red circles.

The dark circles are not ready and become active after a short time. You have one minute.

(must not be bright,
but it is better for fast movements)

## Requirements
Requires OpenCV

https://opencv.org/

for camera usage and eigen 

https://eigen.tuxfamily.org/index.php?title=Main_Page

for calibration.

Gamestart:
The game will start a timer at first start and makes a temporary picture. This 
picture will be used to calibrate. A calibration is nesseccary before first 
gameplay.

## Controls
r = replay
c = calibrate
d = debug
esc = end (gameplay and result)

while calibration:
r = select rectangles again
enter = accept
esc = abort

