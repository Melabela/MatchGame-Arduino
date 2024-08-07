# MatchGame

Interactive game where player matches color and position presented, along the lines of 1980's "Simon" toy.

Designed and Built as my Final Project for "Robotics and Embedded Systems" course, at Mission College.

![Project Photo](_design_docs/07a%20Project%20Assembled.jpg)


## Demonstration Video

Please see **Releases** for video.


## Hardware

### Block Diagram

![Block Diagram](_design_docs/01%20Project%20Identification%20Diagram.png)


### Components List

| Component                 | Purpose                           |
| ------------------------- | --------------------------------- |
| Arduino Uno               | Logic & I/O                       |
| Servo Motor               | Position select                   |
| Fixed color LEDs          | Position targets                  |
| RGB LED                   | Color match select                |
| 74HC595 Shift Register    | Port expansion for Fixed LEDs     |
| Joystick                  | Input: color & position selection |
| Push Button               | Input: confirm selection                   |
| LCD 16x2 module, with I2C | Output: display score & time      |
| Buzzer                    | Output: sounds                    |


## Game Logic

- [Logic Flowchart, page 1](_design_docs/06a%20Logic%20Flowchart%20pg1.png)
- [Logic Flowchart, page 2](_design_docs/06b%20Logic%20Flowchart%20pg2.png)


## Folders

1. `_design_docs` - design files for various parts of the project
2. `_weekly_reports` - weekly reports submitted in class, to show project progress
3. `project_code` - code in C, for Arduino platform (ATmega328 microcontroller)

