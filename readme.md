## Asrover (Astar Rover)

This project started out as a proof of concept to see if it would be possible to apply an A* (A star) style path finding algorythm to a 3D space environment such as KSP worlds.  Taking a script previously written by K4TTEE as a cruise control for manually operated rovers, I expanded the functionality to make it truely autonomous.

## Features

### Speed limiter
Set the desired speed of the rover while cruising between waypoints.

### Set a single destination
The [A* algorythm](https://en.wikipedia.org/wiki/A*_search_algorithm) will find the safest path it can find.  It does this by trying to avoid steep slopes prefering routes between -5 and +15 degrees.

### Automatic speed limiter

The script will monitor the situation of the rover and reduce or maintain speed depending on the environment around it.  Some of these situations include :

* When approaching a waypoint with a deviation of over 5 degrees from current heading.
* When approaching a change of slope angle.  It has a system where it will scan ahead based on the rovers braking distance so that if it detects a sudden change it can efficiently and safely brake before it hits the change.
* While descending down hill it will brake to keep within the set speed limitations or reduce speed for very steep slopes.
* Come to a complete halt and hold if it looses connection to KSC
* Reverse and attempt to go round any objects it hits in it's path.

## Input Keys

Managing the rover is done through Terminal Input with these commands:

    Up arrow      - Increment Latitude by 0.1 degrees
    Down arrow    - Decrease latitude by 0.1 degress
    Left arrow    - Decrease Longitude by 0.1 degrees
    Right arrow   - Increase Longitued by 0.1 degrees

    HOME          - Return arrow vector to vehicle
    END           - Exit the rover manager

    i/I           - Mark the first waypoint for a multi stage journey

    w/W           - Display list of contract waypoints on current body
    1-9           - Select contract waypoint in list

    n/N           - Navigate to next waypoint

    Enter/return  - Execute astar path finding algorythm to destination

    Page Up       - Increase rover speed by 0.5 m/s
    Page down     - Decrease rover speed by 0.5 m/s

## Usage

Clone this solution into the **KSP Root/Ships/Scripts** folder

Due to the size of the scripts it's best to run from Archive unless you have a processor with enough storage (around 30k for both scripts uncompiled).

You can run the solution with
* runpath("/asrover/rover").

### Overhead

Because the astar script is very process intensive it will increase the number of IPU (Instructions Per Update) that KOS can run per tick to 1500 then restore it when the search is completed.  Leaving it at the default 150 will mean that finding routes on larger graphs will take a long time.

The rover management script is designed to run one iterative loop per tick so shouldn't add much overhead while operating.
