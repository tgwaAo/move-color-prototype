# Move Color Prototype

This is a little prototype of a game. 

## DISCLAIMER

THIS GAME DOES NOT CHECK YOUR SURROUNDINGS. PLEASE BE CAREFUL WHILE PLAYING.



## Before gameplay

### Check your surroundings

The game will be played 1-2 meters in front of the camera and used the complete space of the seen image. Please make sure to have enough space!

### Camera choice

This program would normally use the first camera at position 0. If you want to use another camera, create a cam.txt with the necessary number in the directory of the executable.  
You can find the number in the following way:  

#### Windows

1.    Open Powershell
2.    Type  
      ```Get-CimInstance Win32_PnPEntity | ? { $_.service -eq "usbvideo" } | Select-Object -Property PNPDeviceID, Name```
3.    Choose your number

#### Linux

1.    Get a list of devices with  
      `ls /dev/vid*`
2.    Choose only even numbers. E.g. the second camera is 2.

#### Mac

*   Install ffmpeg and run  
    ```ffmpeg -f avfoundation -list_devices true -i \"```  
or  
*  Try out different numbers

## At game start

If you have not configured the game yet, the calibration will start automatically. Please read the calibration section.  
The game will start after the first configuration or at once if a configuration already exists.

## Calibration

*Note: You can always abort the calibration using ESC, but you have to cancel the selection of pixels with ctrl+c.*  
*Note: Bright or shining colors might be better for a detection of fast movements.*

1.    Take a position that shows you, your surroundings and the tracked color. The tracked color should take a bit of the image while your body should keep distance from the camera.
1.    Wait for the countdown to finish.
1.    Select **only** the color with your mouse that has to be tracked. It does not have to be the complete area in the picture, but having only your tracked color is important.  
      Press ctrl+c to cancel and then ESC if you want to cancel the calibration.
1.    Accept with enter.
1.    Select **all** of the tracked color and do not be afraid to have some of the surroundings in the selection. This area will be excluded from the list of negative pixels, so it is only important to have all of the color in it that should be tracked.  
      Press ctrl+c to cancel and then ESC if you want to cancel the calibration.
1.    Accept with enter.
1.    Look at the selection. The pixels for the negative colors in the calibration are red. The pixels for the positive color in the calibration are green.
1.    Accept with enter.
1.    Wait for the calibration to finish.
1.    Look at the result. The negative estimated pixels are red and the positive extimated pixels are green.
1.    Choose your option:
    * Press "s" to save the calibrated values.
    * Press "d" to save the used image and write the calibrated values in a text file for debugging purpose.
    * Press ESC or another key to cancel the calibration.


## Gameplay

### Logic

The game starts with a countdown and then you move a color through the shown window, try to hit the bright green circles and try to avoid bright red circles.  
  
Every circle has 3 states in this order: invisible, dark/not active and bright/active  
A circle will stay invisible and inactive for one second. You then have 3 seconds to hit it in it's bright state.  
When you hit a green circle 1 point is added to your score and then you hit a red circle the score is decreased by 5 points.  
Your final score and the highscore will be shown after 60 seconds from gamestart. If the values are the same, you either have a new highscore or the current score is as high as your highscore.  
  
*Tip: The appearing order of the circles is also the order of them becoming active. You can plan your movement to hit the green circles faster and to avoid hitting the red circles.*  
*Tip: Hit the circle as fast as you can to decrease the time of their active state. That means the same circle will show up more often in one round and can be hit more often.*

### Controls
* "r" for replay
* "c" for entering calibration
* "d" for extracting information for debugging
* ESC for ending a round and showing the current score and the highscore. An additional ESC will end the game.

## Requirements

Requires [OpenCV](https://opencv.org/) for camera usage and [eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) for calibration.

