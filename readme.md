## Asrover (Astar Rover)

This project started out as a proof of concept to see if it would be possible to apply an A* (A star) style path finding algorythm to a 3D space environment such as KSP worlds.  Taking a script previously written by K4TTEE as a cruise control for manually operated rovers, I expanded the functionality to make it truely autonomous.

## Features

### Speed limiter

Set the desired speed of the rover while cruising between waypoints.

### Set a single destination

The A* algorythm will find the safest path it can find.  It does this by trying to avoid steep slopes prefering routes between -5 and +15 degrees.

### Automatic speed limiter

There are 3 scenarios where it will reduce or maintain speed.

* When approaching a waypoint with a deviation of over 5 degrees from current heading.
* When approaching a change of slope angle.  It has a system where it will scan ahead based on the rovers braking distance so that if it detects a sudden change it can efficiently and safely brake before it hits the change.
* While descending down hill it will brake to keep within the set speed limitations

## Input Keys

Managing the rover is done through Terminal Input commands these are :

Up arrow      - Increment Latitude by 0.1 degrees
Down arrow    - Decrease latitude by 0.1 degress
Left arrow    - Increase Longitude by 0.1 degrees
Right arrow   - Decrease Longitued by 0.1 degrees

HOME          - Return arrow vector to vehicle

Enter/return  - Execute astar path finding algorythm to destination

Page Up       - Increase rover speed by 0.5 m/s
Page down     - Decrease rover speed by 0.5 m/s
