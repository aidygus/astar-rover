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
SET speedlimit TO 4. //All speeds are in m/s
SET lastSpeedLimit TO speedlimit.
lock turnlimit to min(1, 0.25 / MAX(0.1,SHIP:GROUNDSPEED)). //Scale the
                   //turning radius based on __current speed
SET TERMINAL:WIDTH TO 30.
SET TERMINAL:HEIGHT TO 40.
SET const_gravity TO BODY:mu / BODY:RADIUS ^ 2.
SET looptime TO 0.01.
SET loopEndTime TO TIME:SECONDS.
SET eWheelThrottle TO 0. // Error between target speed and actual speed
SET iWheelThrottle TO 0. // AccumulATed speed error
SET wtVAL TO 0. //Wheel Throttle Value
SET kTurn TO 0. //Wheel turn value.
SET targetspeed TO 0. //CruISe control starting speed
SET targetHeading TO 90. //Used for autopilot steering
SET NORTHPOLE TO lATlng( 90, 0). //Reference heading
SET AG9 TO FALSE.
SET lockcounter TO 0.
SET nextWaypointHeading TO 0.

set overSpeedDownSlopeBrakeTime to 0.3.
set extremeSlopeAngle to 8.
set overSpeedCruise to 8.
set extraBrakeTime to 0.7.
set cruiseSpeedBrakeTime to 1.
set currentSlopeAngle to 0.
set brakesOn to false.
set lastBrake to -1.
set brakeUseCount to 0.
set currentOverSpeed to overSpeedCruise.
set currentBrakeTime to cruiseSpeedBrakeTime.
SET lasttargetspeed TO 4.
SET stopDistance TO 0.5.
SET gradient TO 0.
SET wpm TO LIST().
SET navpoints TO LIST().

clearscreen.
CLEARVECDRAWS().
sas off.
rcs off.
lights on.
LOCK throttle TO 0.

SET runmode TO 0.
    // Valid runmodes
    //    0 Normal Operation
    //    1 Approaching Waymarker
    //    2 Change of slope angle ahead
    //    3 Moving away from waypoint
    //    4 Very Steep Slope ahead.  Find alternative route.

LOCK __current TO SHIP:GEOPOSITION.
SET __goal TO SHIP:GEOPOSITION.
SET __grid TO SHIP:GEOPOSITION.
nav_marker().
SET route TO LIST().
SET rwaypoint TO -1.
on ag10 { //When the 0 key IS pressed:
    // End the program
    SET runmode TO -1.
    }

until runmode = -1 {
  //UpdATe the compass:
  // I want the heading TO mATch the navball
  // and be out of 360' instead of +/-180'
  // I do thIS by judging the heading relATive
  // TO a lATlng SET TO the north pole
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
      SET targetspeed TO max(-1, min( speedlimit, targetspeed)).
      set upvec to up:vector.
      set velvec to ship:velocity:surface:normalized.
      set dp to vdot(velvec,upvec).
      set currentSlopeAngle to 90 - arccos(dp).


      SET facvec TO SHIP:FACING.

      if route:LENGTH <> 0 AND rwaypoint <> -1  AND rwaypoint+1 < route:LENGTH {
        // IF runmode = 0 { //Govern the rover
        set predicted1 TO body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5))).
        set predicted2 TO body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5)+1)).
        SET heightdiff TO predicted2:TERRAINHEIGHT - predicted1:TERRAINHEIGHT.
        SET distance TO (predicted1:POSITION - predicted2:POSITION):MAG.
        SET angle TO ARCSIN(heightdiff/distance).
        SET gradient TO TAN(currentSlopeAngle).//heightdiff/distance.
        SET pangle TO MAX(currentSlopeAngle,angle) - MIN(currentSlopeAngle,angle).
        PRINT "Predicted Angle : " + round(angle) + "       " AT (2,30).
        PRINT "Difference      : " + round(pangle) + "       " aT (2,31).

        SET stopDistance TO (GROUNDSPEED+0.5)^2 / ( 2 * const_gravity * ( 1 / const_gravity + gradient)).
        if pangle > 5 AND runmode = 0 {          //
          SET runmode TO 2.
          SET lastSpeedLimit TO speedlimit.
          SET lastTargetSpeed TO targetspeed.
          SET speedlimit TO 1.
        } else if runmode = 2 AND pangle < 5 {
          SET runmode TO 0.
          SET speedlimit TO lastSpeedLimit.
          SET targetspeed TO lastTargetSpeed.
        }
        if angle < -10 AND runmode <> 4 {
          SET runmode TO 4.
          SET __grid TO LATLNG(route[rwaypoint-1][0]:LAT,route[rwaypoint-1][0]:LNG).
          LOCK targetHeading TO __grid:HEADING.
        }
        if runmode = 4
        {
          if MAX(nextWaypointHeading,-1*nextWaypointHeading) < 2 {
            SET speedlimit TO lastSpeedLimit.
            SET targetspeed TO lastTargetSpeed.
          }
          if __grid:DISTANCE < 40 AND brakesOn = false {
            SET brakesOn TO true.
            SET speedlimit TO 0.
            SET targetspeed TO 0.
          }
          if GROUNDSPEED < 0.1 {
            SET brakesOn TO true.
            SET speedlimit TO 0.
            SET targetspeed TO 0.
            LOCAL backupRoute IS route.
            LOCAL newGoal IS LATLNG(route[rwaypoint+1][0]:LAT,route[rwaypoint+1][0]:LNG).
            RUNPATH("/asrover/astar","LATLNG",newGoal,false).
            CLEARSCREEN.
            FROM { LOCAL c IS route:LENGTH - 1. } until c = 1 STEP {SET c TO c-1. } DO {
              backupRoute:INSERT(rwaypoint,route[c]).
            }
            SET route TO backupRoute.
            start_navigation().
            SET targetspeed TO 3.
            SET speedlimit TO 3.
            SET runmode TO 1.
          }
        }
        SET headingDifference TO route[rwaypoint][0]:HEADING - cHeading.
        SET headingDifference TO MAX(headingDifference,-1*headingDifference).
        SET nextWaypointHeading TO route[rwaypoint+1][0]:HEADING - cHeading.
        if __grid:DISTANCE < 50 AND MAX(nextWaypointHeading,-1*nextWaypointHeading) > 5 AND runmode = 0 and rwaypoint <> 0 {
          SET runmode TO 1.
          SET lastSpeedLimit TO speedlimit.
          SET lastTargetSpeed TO targetspeed.
          SET speedlimit TO 3.
        }
        if runmode = 3 {
          if route[rwaypoint-1][0]:DISTANCE > 30 OR headingDifference <= 2 {
            SET speedlimit TO lastSpeedLimit.
            SET targetspeed TO lastTargetSpeed.
            SET runmode TO 0.
          }
        }
        if route[rwaypoint][0]:DISTANCE < 20 {
          SET rwaypoint TO rwaypoint + 1.
          if rwaypoint < route:LENGTH {
            SET __grid TO LATLNG(route[rwaypoint][0]:LAT,route[rwaypoint][0]:LNG).
            // LOCK WHEELSTEERING TO route[rwaypoint][0].
            LOCK targetHeading TO __grid:HEADING.
            SET runmode TO 3.
          }
        }
      } else {
        set route TO LIST().
        SET targetspeed TO 0.
        set rwaypoint TO -1.
        SET runmode TO 0.
        IF navpoints:LENGTH <> 0 {
          LOCAL gl IS navpoints[0].
          navpoints:REMOVE(0).
          CLEARSCREEN.
          RUNPATH("/asrover/astar","LATLNG",gl,false).
          start_navigation().
        } else {
          PRINT "         Rover has arrived at location                 " AT (0,1).
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
      ELSE IF targetspeed < 0 { //ELSE IF we're going backwards
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
      SET K TO TERMINAL:INPUT:GETCHAR().

      IF K = TERMINAL:INPUT:UPCURSORONE {
        SET __goal TO LATLNG(__goal:LAT+0.1,__goal:LNG).
        nav_marker().
      }
      IF K = TERMINAL:INPUT:DOWNCURSORONE {
        SET __goal TO LATLNG(__goal:LAT-0.1,__goal:LNG).
        nav_marker().
      }
      IF K = TERMINAL:INPUT:LEFTCURSORONE {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG-0.1).
        nav_marker().
      }
      IF K = TERMINAL:INPUT:RIGHTCURSORONE {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG+0.1).
        nav_marker().
      }
      IF K = TERMINAL:INPUT:ENTER {
        if navpoints:LENGTH = 0 {
          LOCAL gl IS __goal
        } else {
          LOCAL gl IS navpoints[0].
          navpoints:REMOVE(0).
        }
        CLEARSCREEN.
        RUNPATH("/asrover/astar","LATLNG",gl,false).
        start_navigation().
      }
      IF K = TERMINAL:INPUT:PAGEUPCURSOR {
        set speedlimit to speedlimit + 0.5.
        SET targetspeed TO speedlimit.
        SET lastSpeedLimit TO speedlimit.
        SET lasttargetspeed TO targetspeed.
        }
      if K = TERMINAL:INPUT:PAGEDOWNCURSOR {
        set speedlimit to speedlimit - 0.5.
        SET targetspeed TO speedlimit.
        SET lastSpeedLimit TO speedlimit.
        SET lastTargetSpeed TO targetspeed.
      }
      IF K = TERMINAL:INPUT:HOMECURSOR {
        SET __goal TO SHIP:GEOPOSITION.
        nav_marker().
        SET targetspeed TO 0.
        SET route TO LIST().
        SET rwaypoint TO -1.
        //Prevent decrease IF we are increasing
      }
      IF K = TERMINAL:INPUT:INSERTCURSOR {
        navpoints:ADD(__goal).
        waypoint_marker().
      }
      IF K = TERMINAL:INPUT:ENDCURSOR {
        SET runmode TO -1.
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


    PRINT "Target Speed:   " + ROUND( targetspeed, 1) + "        " AT (2, 3).
    PRINT "Speed Limit:    " + ROUND( speedlimit, 1) + "        " AT (2, 4).
    PRINT "Surface Speed:  " + ROUND( GROUNDSPEED, 1) + "        " AT (2, 5).

    PRINT "Pilot Throttle: " + ROUND( SHIP:CONTROL:PILOTWHEELTHROTTLE, 2) + "        " AT (2, 7).
    PRINT "Kommanded tVAL: " + ROUND( wtVAL, 2) + "        " AT (2, 8).
    PRINT "Pilot Turn:     " + ROUND( SHIP:CONTROL:PILOTWHEELSTEER, 2) + "        " AT (2, 9).
    PRINT "Kommanded Turn: " + ROUND( kTurn, 2) + "        " AT (2, 10).

    PRINT "Target Heading: " + ROUND( targetheading, 2) + "        " AT (2, 12).
    PRINT "CurrentHeading: " + ROUND( cheading, 2) + "        " AT (2, 13).
    PRINT "CruISe Control: " + AG1 + "   " AT (2, 14).

    PRINT "Waypoint Dist   : " + ROUND(__grid:DISTANCE, 2) + "        " AT (2, 16).
    PRINT "Waypoints     : " + route:LENGTH + "        " AT (2, 17).
    PRINT "Current WP    : " + rwaypoint + "        " AT (2, 18).

    PRINT "E: " + ROUND(eWheelThrottle,2)+ "   "  AT ( 2, 20).
    PRINT "I: " + ROUND(iWheelThrottle,2) + "   " AT (10,20).
    PRINT "WTVAL " + ROUND(WTVAL,2) + "     " AT (2 ,21).

    IF DEFINED route AND route:LENGTH <> 0 AND rwaypoint <> route:LENGTH-1 {
      PRINT "Next Target     : " + round(MAX(route[rwaypoint+1][0]:HEADING, -1*route[rwaypoint+1][0]:HEADING)) + "         " AT (2, 24).
      PRINT "Next HEading    : " + round(nextWaypointHeading) + "            " AT (2,25).
    }
    PRINT "Current Angle   : " + round(currentSlopeAngle,2) + "          " AT (2,28).

    PRINT "Distance to goal: " + round(__goal:DISTANCE) + "        " AT (2,33).
    PRINT "Runmode         :" + Runmode + "            " AT (2, 34).

        PRINT "Stopping Dist   : " + round(stopDistance,4) + "        " AT (2,36).
        PRINT "Gradient        : " + round(gradient,4) + "      " AT (2,37).

    SET looptime TO TIME:SECONDS - loopEndTime.
    SET loopEndTime TO TIME:SECONDS.

    }

    FUNCTION nav_marker {
      SET vg TO VECDRAWARGS(
                  __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  Green, "", 1, true,50).
    }

    FUNCTION waypoint_marker {
      SET wpm:ADD(VECDRAWARGS(
                  __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+800),
                  __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+800),
                  Blue, "", 1, true,50)).
    }

    FUNCTION start_navigation()
    {
      CLEARSCREEN.
      SET rwaypoint TO 0.
      SET __grid TO LATLNG(route[rwaypoint][0]:LAT,route[rwaypoint][0]:LNG).
      LOCK targetHeading TO __grid:HEADING.
      BRAKES OFF.
      SET AG1 TO TRUE.
      SET targetspeed TO speedlimit.
      SET lastSpeedLimit TO speedlimit.
      SET lasttargetspeed TO targetspeed.
    }
