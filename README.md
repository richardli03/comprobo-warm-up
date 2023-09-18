# comprobo-warm-up

In your github repository, create a markdown file called README.md to serve as documentation for your project. Your writeup should answer the following questions. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).


    - For each behavior, describe the problem at a high-level. Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.
    - For the finite state controller, what was the overall behavior. What were the states? What did the robot do in each state? How did you combine and how did you detect when to transition between behaviors?  Consider including a state transition diagram in your writeup.
    -  How was your code structured? Make sure to include a sufficient detail about the object-oriented structure you used for your project.
    -  What if any challenges did you face along the way?
    -  What would you do to improve your project if you had more time?
    -  What are the key takeaways from this assignment for future robotic programming projects? For each takeaway, provide a sentence or two of elaboration.


## Teleop
The `Teleop` node is designed to accept keyboard inputs from the user and translate that to motion of the robot. My implementation of `Teleop` uses WASD as movement keys 

    - `W` stops the robot from turning and moves the robot forwards
    - `S` stops the robot from turning and moves the robot backwards 
    - `A` stops the robot's forward/backward movement and turns the robot left (relative to the direction it is currently facing) 
    - `D` stops the robot's forward/backward movement and turns the robot right (relative to the direction it is currently facing)
    - `Q` stops all forward/backward movement and turning. 

![State Diagram](assets/TELEOP.png)

The implementation strategy for this node primarily relys on a `get_key` function, which acquires keyboard inputs from `stdin` and returns it to the script. The other piece of the puzzle is the main loop, which blocks until it receives keyboard input, then sets the robot's linear and angular velocities appropriately. 

The only special case to consider is `CTRL+C`, normally used to exit a program. Because our `get_key` function completely hijacks stdin, we need to handle a keyboard interrupt manually. We do this by catching the input and throwing it to the script.

This script achieves its purpose, in proving that I could communicate with the robot and control its motion. It has many flaws, but perhaps the biggest one is that the motion of robot isn't particularly intuitive. Generally, WASD motion *considers how long the key is held*. That is to say, I wouldn't tap `W` to make the robot move forward forever (the current behavior), but I should *hold* `W` to move. That implementation is a little bit outside of the scope of this project, though it would greatly improve the implementation of `Teleop`

## Drive-Square

## Wall-Following

## Person-Following

## Obstacle Avoidance
