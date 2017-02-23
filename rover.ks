// rover.ks
// Original direction and speed routines written by KK4TEE
// Updated with waypoint management by aidygus
// License: GPLv3
//
// ThIS program provides waypoint functionality
// using an astar based algorythm to calculate
// safest possible route from start to goal
// and monitoring for fully automated rovers

PARAMETER debug IS true.

lock turnlimit to min(1, 0.1 / MAX(0.1,SHIP:GROUNDSPEED)). //Scale the
                   //turning radius based on __current speed
SET TERMINAL:WIDTH TO 50.
SET TERMINAL:HEIGHT TO 40.
SET const_gravity TO BODY:mu / BODY:RADIUS ^ 2.
SET looptime TO 0.01.
SET loopEndTime TO TIME:SECONDS.
SET eWheelThrottle TO 0. // Error between target speed and actual speed
SET iWheelThrottle TO 0. // AccumulATed speed error
SET wtVAL TO 0. //Wheel Throttle Value
SET kTurn TO 0. //Wheel turn value.
SET targetspeed TO 0. //CruISe control starting speed
SET lastTargetSpeed TO 4.
SET targetHeading TO 90. //Used for autopilot steering
SET NORTHPOLE TO lATlng( 90, 0). //Reference heading
SET AG9 TO FALSE.
SET lockcounter TO 0.
SET nextWaypointHeading TO 0.
SET spc TO "      ".

set overSpeedDownSlopeBrakeTime to 0.3.
set extremeSlopeAngle to 8.
set overSpeedCruise to 8.
set extraBrakeTime to 0.7.
set cruiseSpeedBrakeTime to 1.
set currentSlopeAngle to 0.
set brakesOn to TRUE.
set lastBrake to -1.
set lastEvent TO -1.
set brakeUseCount to 0.
set currentOverSpeed to overSpeedCruise.
set currentBrakeTime to cruiseSpeedBrakeTime.
SET stopDistance TO 0.5.
SET gradient TO 0.
SET wpm TO LIST().
SET navpoints TO LIST().
SET contractWayPoints TO LIST().

clearscreen.
CLEARVECDRAWS().
sas off.
rcs off.
lights on.
BRAKES ON.
LOCK throttle TO 0.

SET runmode TO 9.
    // Valid runmodes
    //    0  Normal Operation
    //    1  Approaching Waymarker
    //    2  Change of slope angle ahead
    //    3  Moving away from waypoint
    //    4  Very Steep Slope ahead.  Find alternative route.
    //    5  Has hit an obstacle.
    //    6  Attempting to move round obstacle.
    
    //    9  Display Menu
    //    10 Select Waypoint
    //    11 Do Science
    //    12 No Connection

LOCK __current TO SHIP:GEOPOSITION.
SET __goal TO SHIP:GEOPOSITION.
SET __grid TO SHIP:GEOPOSITION.
nav_marker().
SET route TO LIST().
SET rwaypoint TO -1.

display_HUD().
until runmode = -1 {
  //Update the compass:
  // I want the heading TO match the navball
  // and be out of 360' instead of +/-180'
  // I do this by judging the heading relative
  // TO a lAT/LNG SET TO the north pole
  IF northPole:bearing <= 0 {
      SET cHeading TO ABS(northPole:bearing).
      }
  ELSE {
      SET cHeading TO (180 - northPole:bearing) + 180.
      }


  // SET v2 TO VECDRAWARGS(
  //             predicted:ALTITUDEPOSITION(predicted:TERRAINHEIGHT+100),
  //             predicted:POSITION - predicted:ALTITUDEPOSITION(predicted:TERRAINHEIGHT+100),
  //             Green, "", 1, true,5).
      //Wheel Throttle:
      SET targetspeed TO targetspeed + 0.1 * SHIP:CONTROL:PILOTWHEELTHROTTLE.
      SET targetspeed TO max(-1, targetspeed).
      set upvec to up:vector.
      set velvec to ship:velocity:surface:normalized.
      set dp to vdot(velvec,upvec).
      set currentSlopeAngle to 90 - arccos(dp).

      SET facvec TO SHIP:FACING.

      if route:LENGTH <> 0 AND rwaypoint <> -1  AND rwaypoint+1 < route:LENGTH {
        // IF runmode = 0 { //Govern the rover
        LOCAL predicted1 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,sISpDistance+5))).
        LOCAL predicted2 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,sISpDistance+5)+1)).
        LOCAL heightdiff IS predicted2:TERRAINHEIGHT - predicted1:TERRAINHEIGHT.
        LOCAL distance IS (predicted1:POSITION - predicted2:POSITION):MAG.
        SET angle TO ARCSIN(heightdiff/distance).
        SET gradient TO TAN(currentSlopeAngle).//heightdiff/distance.
        SET pangle TO MAX(currentSlopeAngle,angle) - MIN(currentSlopeAngle,angle).
        PRINT round(angle) + spc AT (20,21).
        PRINT round(pangle) + spc AT (20,23).

        SET stopDistance TO (GROUNDSPEED+0.5)^2 / ( 2 * const_gravity * ( 1 / const_gravity + gradient)).

        IF GROUNDSPEED < 0.1 AND targetspeed <> 0 AND (TIME:SECONDS - lastEvent) > 20 {
          SET runmode to 5.
          SET targetspeed TO -1.
          SET lastEvent TO TIME:SECONDS.
          LOCK targetHeading TO (__grid:HEADING - 90).
        }
        ELSE if pangle > 5 AND runmode = 0 {          //
          SET runmode TO 2.
          set_speed(1).
        } else if runmode = 2 AND pangle < 5 {
          SET runmode TO 0.
          restore_speed().
        }
        if angle < -25 AND runmode <> 4 {
          set_speed(2).
        //   SET runmode TO 4.
        //   SET __grid TO LATLNG(route[rwaypoint-1][0]:LAT,route[rwaypoint-1][0]:LNG).
        //   LOCK targetHeading TO __grid:HEADING.
        }

        IF ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCONNECTION(SHIP) = FALSE AND runmode <> 12 {
          SET runmode to 12.
          set_speed(0).
          BRAKES ON.
          SET brakesOn TO TRUE.
        } else if ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCONNECTION(SHIP) AND runmode = 12 {
          SET runmode TO 0.
          if route:LENGTH <> 0 {
            restore_speed().
            SET brakesOn TO FALSE.
          }
        }

        if runmode = 4 AND angle > -15
        {
            restore_speed().
        }
        LOCAL headingDifference IS route[rwaypoint]:HEADING - cHeading.
        SET headingDifference TO MAX(headingDifference,-1*headingDifference).
        SET nextWaypointHeading TO route[rwaypoint+1]:HEADING - cHeading.
        if __grid:DISTANCE < 50 AND MAX(nextWaypointHeading,-1*nextWaypointHeading) > 5 AND runmode = 0 and rwaypoint <> 0 {
          SET runmode TO 1.
          set_speed(3).
        }
        if runmode = 3 {
          if route[rwaypoint-1]:DISTANCE > 30 AND headingDifference < 2 {
            restore_speed().
            SET runmode TO 0.
          }
        }
        if route[rwaypoint]:DISTANCE < MAX(5,stopDistance) {
          next_waypoint().
        }
        if runmode = 5 AND (TIME:SECONDS - lastEvent) > 20 {
          SET runmode TO 6.
          SET targetspeed TO 2.
          SET lastEvent TO TIME:SECONDS.
        } else if runmode = 6 AND (TIME:SECONDS - lastEvent) > 20 {
          LOCK targetHeading TO __grid:HEADING.
          SET runmode TO 3.
        }
      } else {
        IF navpoints:LENGTH <> 0 AND rwaypoint <> -1{
          set route TO LIST().
          SET targetspeed TO 0.
          set rwaypoint TO -1.
          SET runmode TO 0.
          LOCAL gl IS navpoints[0].
          navpoints:REMOVE(0).
          RUNPATH("/asrover/astar","LATLNG",gl,false).
          start_navigation().
        } else {
          PRINT "---{   Rover has arrived at location  }---         " + spc AT (2,1).
          SET targetspeed TO 0.
          BRAKES ON.
          UNLOCK targetheading.
        }
      }


      IF targetspeed > 0 { //IF we should be going forward
        if brakesOn = false {
          brakes off.
        }
        SET eWheelThrottle TO targetspeed - GROUNDSPEED.
        SET iWheelThrottle TO min( 1, max( -1, iWheelThrottle +
                                            (looptime * eWheelThrottle))).
        SET wtVAL TO eWheelThrottle + iWheelThrottle.//PI controler
        IF GROUNDSPEED < 5 {
          //Safety adjustment TO help reduce roll-back AT low speeds
          SET wtVAL TO min( 1, max( -0.2, wtVAL)).
        }
      }
      ELSE IF targetspeed < 0 AND runmode = 0 { //ELSE IF we're going backwards
        SET wtVAL TO SHIP:CONTROL:PILOTWHEELTHROTTLE.
        SET targetspeed TO 0. //Manual reverse throttle
        SET iWheelThrottle TO 0.
      }
      ELSE { // IF value IS out of range or zero, stop.
        SET wtVAL TO 0.
        brakes on.
      }

      // IF brakes = 1 AND brakesOn = false { //DISable cruISe control IF the brakes are turned on.
      //     SET targetspeed TO 0.
      //     }
        //Steering:
        IF AG1 { //ActivATe autopilot IF Action group 1 IS on
            SET errorSteering TO (targetheading - cHeading).
            IF errorSteering > 180 { //Make sure the headings make sense
                SET errorSteering TO errorSteering - 360.
                }
            ELSE IF errorSteering < -180 {
                SET errorSteering TO errorSteering + 360.
                }
            SET desiredSteering TO -errorSteering / 20.
            SET kturn TO min( 1, max( -1, desiredSteering)) * turnlimit.
            }
        ELSE {
            SET kturn TO turnlimit * SHIP:CONTROL:PILOTWHEELSTEER.
        }

        when abs(GROUNDSPEED) > targetspeed then {  // borrowed from gaiiden / RoverDriver https://github.com/Gaiiden/RoverDriver/blob/master/begindrive.txt
          if brakesOn = false {
            set brakesOn to true.
            brakes on.
            set brakeUseCount to brakeUseCount + 1.

            if abs(currentSlopeAngle) > extremeSlopeAngle {
              set currentBrakeTime to overSpeedDownSlopeBrakeTime + extraBrakeTime.
            }

            set lastBrake to time:seconds.
          }.
          preserve.
        }.

        // do we need to disable the brakes?
        when brakesOn = true and (abs(GROUNDSPEED) <= targetspeed OR time:seconds - lastBrake >= currentBrakeTime) then {
          brakes off.
          set brakesOn to false.
          preserve.
        }.
    // }


    //Handle User Input using action groups
    IF TERMINAL:INPUT:HASCHAR {
      LOCAL K IS TERMINAL:INPUT:GETCHAR().
      LOCAL N IS K:TONUMBER(-99).
      IF K = TERMINAL:INPUT:UPCURSORONE {
        SET __goal TO LATLNG(__goal:LAT+0.1,__goal:LNG).
        nav_marker().
      }
      ELSE IF K = TERMINAL:INPUT:DOWNCURSORONE {
        SET __goal TO LATLNG(__goal:LAT-0.1,__goal:LNG).
        nav_marker().
      }
      ELSE IF K = TERMINAL:INPUT:LEFTCURSORONE {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG-0.1).
        nav_marker().
      }
      ELSE IF K = TERMINAL:INPUT:RIGHTCURSORONE {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG+0.1).
        nav_marker().
      }
      ELSE IF K = TERMINAL:INPUT:RETURN {
        if navpoints:LENGTH = 0 {
          RUNPATH("/asrover/astar2","LATLNG",__goal,false).
        } else {
          RUNPATH("/asrover/astar2","LATLNG",navpoints[0],false).
          navpoints:REMOVE(0).
        }
        start_navigation().
        PRINT "  ---{   Navigating to LAT " + round(__goal:LAT,1) +" : LNG " + round(__goal:LNG,1) +  "   }---" AT (0,1).
      }
      ELSE IF K = TERMINAL:INPUT:PAGEUPCURSOR {
        SET targetspeed TO targetspeed + 0.5.
        SET lasttargetspeed TO targetspeed.
        }
      ELSE if K = TERMINAL:INPUT:PAGEDOWNCURSOR {
        SET targetspeed TO targetspeed - 0.5.
        SET lastTargetSpeed TO targetspeed.
      }
      ELSE IF K = TERMINAL:INPUT:HOMECURSOR {
        SET __goal TO SHIP:GEOPOSITION.
        nav_marker().
        SET targetspeed TO 0.
        SET route TO LIST().
        SET rwaypoint TO -1.
        //Prevent decrease IF we are increasing
      }
      ELSE IF K = "i" OR K = "I" {
        navpoints:ADD(__goal).
        waypoint_marker().
      }
      ELSE IF K = "w" OR K = "W" {
        CLEARSCREEN.
        LOCAL WPS IS ALLWAYPOINTS().
        SET runmode TO 10.
        FOR WP IN WPS {
          IF WP:BODY:NAME = BODY:NAME {
            contractWayPoints:ADD(WP).
          }
        }
      }
      ELSE IF K = "n" OR K = "N" {
        next_waypoint().
      }
      ELSE IF K = TERMINAL:INPUT:ENDCURSOR {
        SET runmode TO -1.
        CLEARVECDRAWS().
      } ELSE IF N <> -99 {
        if N <= contractWayPoints:LENGTH {
          SET runmode TO 0.
          LOCAL w IS contractWayPoints[N-1].
          RUNPATH("/asrover/astar2","WAYPOINT",w:NAME,false).
          start_navigation().
          PRINT "  ---{   Navigating to " + w:NAME +"   }---" AT (0,1).
        }
      } ELSE {
        PRINT "# UNKNOWN COMMAND # " AT (2,TERMINAL:HEIGHT - 2).
      }
    }

      IF targetHeading > 360 {
        SET targetHeading TO targetHeading - 360.
      }
      ELSE IF targetHeading < 0 {
        SET targetHeading TO targetHeading + 360.
      }

    SET SHIP:CONTROL:WHEELTHROTTLE TO WTVAL.
    SET SHIP:CONTROL:WHEELSTEER TO kTurn.

    if runmode = 10 {
      PRINT "Press a number to select a waypoint" AT (2, 3).
      LOCAL y IS 1.
      FOR c IN contractWayPoints {
        PRINT "(" + y + ") " + c:NAME +"      " AT (4,y+4).
        LOCAL p IS c:GEOPOSITION.
        PRINT " " + round((p:DISTANCE/1000),2) + " km" AT (25,y+4).
        LOCAL y IS y+1.
      }
    } else {
      PRINT ROUND( targetspeed, 1) + spc AT (20, 6).
      PRINT ROUND( GROUNDSPEED, 1) + spc AT (20, 7).
      PRINT round(__goal:DISTANCE) + spc AT (20, 8).

      PRINT ROUND( targetheading, 2) + spc AT (20, 12).
      PRINT ROUND( cheading, 2) + spc AT (20, 13).

      IF DEFINED route AND route:LENGTH <> 0 AND rwaypoint <> route:LENGTH-1 {
        PRINT round(MAX(route[rwaypoint+1]:HEADING, -1*route[rwaypoint+1]:HEADING)) + spc AT (20, 14).
        PRINT round(nextWaypointHeading) + spc AT (20,15).
      }
      PRINT ROUND(__grid:DISTANCE, 2) + spc AT (20, 16).
      PRINT route:LENGTH + spc AT (20, 17).
      PRINT rwaypoint + spc AT (20, 18).

      PRINT round(currentSlopeAngle,2) + spc AT (20,20).

      PRINT round(stopDistance,4) + spc AT (20,24).
      PRINT round(gradient,4) + spc AT (20,25).

      PRINT Runmode + spc AT (20, 27).
      PRINT AG1 + "   " AT (20, 28).

      PRINT ROUND( wtVAL, 2) + spc AT (20, 30).
      PRINT ROUND( kTurn, 2) + spc AT (20, 31).

      PRINT ROUND(eWheelThrottle,2)  AT ( 6, 33).
      PRINT ROUND(iWheelThrottle,2) AT (14,33).
      PRINT ROUND(WTVAL,2) + spc AT (20 ,34).
    }
    SET looptime TO TIME:SECONDS - loopEndTime.
    SET loopEndTime TO TIME:SECONDS.
    }

    FUNCTION display_HUD {
      PRINT "Target Speed    :" AT (2,6).
      PRINT "Surface Speed   :" AT (2,7).
      PRINT "Distance to goal:" AT (2,8).

      PRINT "Target Heading  :" AT (2,12).
      PRINT "CurrentHeading  :" AT (2,13).
      PRINT "  Next Heading  :" AT (2,14).
      PRINT "  Next Bearing  :" AT (2,15).

      PRINT "Waypoint Dist   :"  AT (2,16).
      PRINT "Waypoints       :" AT (2,17).
      PRINT "Current WP      :" AT (2,18).

      PRINT "Current Angle   :" AT (2,20).
      PRINT "Predicted Angle :" AT (2,21).

      PRINT "Difference      :" AT (2,23).
      PRINT "Stopping Dist   :" AT (2,24).
      PRINT "Gradient        :" AT (2,25).

      PRINT "Runmode         :" AT (2,27).
      PRINT "Cruise Control  :" AT (2,28).

      PRINT "Commanded tVAL  :" AT (2,30).
      PRINT "Commanded Turn  :" AT (2,31).

      PRINT "E:" AT ( 2, 33).
      PRINT "I:" AT (10,33).
      PRINT "WTVAL " AT (2 ,35).
    }

    FUNCTION nav_marker {
      LOCAL vg IS VECDRAWARGS(
                  __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  Green, "", 1, true,50).
    }

    FUNCTION waypoint_marker {
      wpm:ADD(VECDRAWARGS(
                  __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+800),
                  __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+800),
                  Blue, "", 1, true,50)).
    }

    FUNCTION start_navigation
    {
      SET TERMINAL:WIDTH TO 50.
      SET TERMINAL:HEIGHT TO 40.
      CLEARSCREEN.
      display_HUD().
      SET rwaypoint TO 0.
      if route:LENGTH <> 0 {
        SET __grid TO LATLNG(route[rwaypoint]:LAT,route[rwaypoint]:LNG).
        LOCK targetHeading TO __grid:HEADING.
        BRAKES OFF.
        LIGHTS ON.
        SET AG1 TO TRUE.
        restore_speed().
        SET lastEvent TO TIME:SECONDS.
      }
    }

    FUNCTION set_speed
    {
      PARAMETER spd.
      if spd < targetspeed {
        SET lastTargetSpeed TO targetspeed.
        SET targetspeed TO spd.
      }
    }

    FUNCTION restore_speed
    {
      SET targetspeed TO lastTargetSpeed.
    }

    FUNCTION next_waypoint
    {
      SET rwaypoint TO rwaypoint + 1.
      if rwaypoint < route:LENGTH {
        SET __grid TO LATLNG(route[rwaypoint]:LAT,route[rwaypoint]:LNG).
        // LOCK WHEELSTEERING TO route[rwaypoint].
        LOCK targetHeading TO __grid:HEADING.
        SET runmode TO 3.
      }
    }
