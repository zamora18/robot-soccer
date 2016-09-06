Vision code for Chicken McThuggets

Uses threading to process the frames coming from the IP camera in order to determine location of the ball and 4 robots.

On startup asks for how many Allies and how many Opponents.
  - based on these numbers it prompts for the colors of the allies and opponents for flexibility
    - in Winter 2016 it was Purple and blue vs Orange and green.
  - it expects a string identifying the color
    - p = Purple
    - b = blue
    - o = ornage
    - g = green
    - (there is a 5th color bc, it was a really terrible blue, but we were able to calibrate it and see for testing sake)
  - the colors need to be calibrated each time for a competition
    - they can be changed by the sliders that start when the program starts and saved by entering the numbers in VisionObject.cpp


The window will appear with the robots being tracked. the lines are the way they are facing (small blob of color in the front)


The ball is circled by a red circle.

pressing the Spacebar sends a "go" signal to the robots to start playing

pressing the "a" key will switch the field to be able to play away

pressing the numkeys 1-5 will toggle views to see how well the colors are doing.
 - IMPORTANT NOTE - these must be OFF when playing, having the black and white windows will slow the vision code and you WILL lose frames


