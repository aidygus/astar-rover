# Asrover (Astar Rover)

This project started out as a proof of concept to see if it would be possible to apply an [A* (A Star)](https://en.wikipedia.org/wiki/A*_search_algorithm) style path finding algorythm to a 3D space environment such as KSP planets.  The script works by mathematically disecting the world into a grid where each cell is roughly 100m per side and attempting to validate if a viable safe route exists by setting what could be considered as safe Maximum and Minimum slopes to negotiate then comparing the calculated slope value between the current cell and neighbour. There are no knowns and all validations are performed on the fly. While calculating the script will display on the screen it's results.

Key

     S  -  Start cell
     G  -  Goal
     .  - Validated cell which is safe to visit
     ^  - Slope exceeds Maximum value
     v  - Slope exceeds Minimum value
     !  - Cell is below sea level therefore out of bounds

## Features

### Set a single destination
The A* algorythm will attempt to find the safest path between the vessel's current position and the specified goal.  It does this by trying to avoid steep slopes based on the Minimum and Maximum values specified during configuration.

You can manually specify a location to navigate to via the cursor keys to move the goal indication vector, or you can select a waypoint if in a contract situation or use waypoint manager and have specified a custom point.

### Automatic speed limiter

The script will monitor the situation of the rover and reduce or maintain speed depending on the environment around it.  Some of these situations include :

* When approaching a waypoint with a deviation of over 5 degrees from current heading.
* When approaching a change of slope angle.  It has a system where it will scan ahead based on the rovers braking distance so that if it detects a sudden change it can efficiently and safely brake before it hits the change.
* While descending down hill it will brake to keep within the set speed limitations or reduce speed for very steep slopes to within the stop distance of the default target speed.
* Come to a complete halt and hold if it looses connection to KSC or is low on electric charge.
* Reverse and attempt to go round any objects it hits in it's path.

### Configuration

There are some settings that may not fit all environments.  Such as what classifies as an acceptable Minimum or Maximum slope that a rover could negotiate or what is a safe cruising speed so I added a setup script to help make this script a bit more dynamic but still allow you to tailor the rover for whichever body it's being deployed to.

This setup script can be ran with **runpath("0:/astar-rover/setup").**.  

If you run the /astar-rover/rover.ks on a rover that hasn't been set up yet, this utility will be called allowing you to customize how the rover behaves.  The setup can be ran at any time by pressing C in the terminal window.

The rover script can be ran from either the local processor or from the archive, but the configuration settings and logging information will be store locally to the rover so you can run from archive with multiple rovers and have each with their own settings.

In order to run the script locally you will need 20k space on the KOS processor to store the compiled scripts and configuration settings.

You are curently able to set these 5 values :

    Minimum slope   - The sharpest descent in angles that astar will decide is or isn't a valid neighbour.
    Maximum slope   - As above but for ascending slopes
    IPU             - These are the number of KRisc instructions that KOS can run per physics tick.  The higher the number, the faster A* will find a route
    Default Speed   - The normal cruise speed of the rover between waypoints.
    Turn Limit      - Effects how quickly the rover will turn.  For lower gravity planets, reduce it to make sure it doesn't tip while turning.

All these settings will be stored in the local drive within 1:/config/settings.json.

If you want to run the rover from the local kOSProcessor rather than from Archive then you will need at least 15k capacity in order to store the compiled artefacts.

### Run Science experiments

Tag all parts that contain Experiments with **Skience** and if you select that menu, it will display a list of parts which you can select.  The rover will then run the experiment and transmit it to KSC.  Handy for contract situations where you may have to run the same experiment 4 or 5 times.  Currently supports stock and DMModuleScienceAnimate part.

### Odometer

Will keep a record of the distance the rover has travelled.

### sounds

When the status of the rover changes, it will play a sound to let you know.  Depending where it is and what it's driving over, it can be quite vocal at times.

## Input Keys

Managing the rover is done through Terminal Input with these commands:

    Up arrow      - Increment Latitude by 0.1 degrees
    Down arrow    - Decrease latitude by 0.1 degress
    Left arrow    - Decrease Longitude by 0.1 degrees
    Right arrow   - Increase Longitued by 0.1 degrees

    HOME          - Return arrow vector to vehicle or return to main display if on a sub feature
    END           - Exit the rover manager

    i/I           - Mark the first waypoint for a multi stage journey

    w/W           - Display list of contract waypoints on current body
    1-9           - Select contract waypoint in list or science part

    n/N           - Navigate to next waypoint in route

    c/C           - Run the configuration utility stored on Archive.
    s/S           - Perform Science experiments
    r/R           - Refresh screen and display main HUD
    v/V           - Clear Vectors from screen
    h/H           - Come to a complete stop

    c/C           - Run the configuration utility stored on Archive.
    s/S           - Perform Science experiments
    r/R           - Refresh screen and display main HUD

    Enter/return  - Execute astar path finding algorythm to destination

    Page Up       - Increase rover speed by 0.5 m/s
    Page down     - Decrease rover speed by 0.5 m/s

## Usage

Clone this solution into the **KSP Root/Ships/Script** folder keeping the files in their own sub folder.

Due to the size of the scripts it's best to run from Archive unless you have a processor with enough storage (around 20K for both scripts compiled).

You can run the solution with

* runpath("/astar-rover/rover").

Or to run the setup utility

* runpath("0:/astar-rover/setup").

### Setup utility

This feature can be ran directly or through the main rover screen by pressing *C*.  When changing from within the rover, the settings will be available straight away and you won't need to reboot.

It can be used for the following functions :

1. Initializing the rover by creating the config folder to store settings and logging information, a boot folder with the bootstrap file and if there is enough space (at least 20k) compile the scripts to the local processor store.
2. Configure settings allows you to customize settings which will effect the way the rover behaves and finds routes.  It saves automatically when you change a value
3. Reboot rover
4. Reset to factory default will remove all astar-rover files from the local processor store

### Overhead

Because the A* script is very process intensive it will temporarily increase the number of IPU (Instructions Per Update) that KOS can run per physics tick then restore it to the original value when the search is completed.  Leaving it at the default 150 will mean that finding routes will take a long time.  How many instructions can be ran can be configured in the setup utility in steps of 500, but it is reccomended to leave it at 2000 so that it can calculate the route in the shortest time.

The rover management script is designed to run one iterative loop per physics tick so shouldn't add much overhead while operating.
