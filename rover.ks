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
SET const_gravity TO BODY:mu / BODY:RADIUS ^ 2.

if EXISTS("1:/config/settings.json") = FALSE {
  runpath("0:/astar-rover/setup").
  REBOOT.
} else {
  SET settings TO READJSON("1:/config/settings.json").
}

lock turnlimit to min(1, (settings["TurnLimit"] * const_gravity) / SHIP:GROUNDSPEED). //Scale the
                   //turning radius based on __current speed
SET TERMINAL:WIDTH TO 50.
SET TERMINAL:HEIGHT TO 40.
SET looptime TO 0.01.
SET loopEndTime TO TIME:SECONDS.
SET eWheelThrottle TO 0. // Error between target speed and actual speed
SET iWheelThrottle TO 0. // AccumulATed speed error
SET wtVAL TO 0. //Wheel Throttle Value
SET kTurn TO 0. //Wheel turn value.
SET targetspeed TO 0. //CruISe control starting speed
SET lastTargetSpeed TO settings["DefaultSpeed"].
SET slopeSpeed TO lastTargetSpeed.
SET targetHeading TO 90. //Used for autopilot steering
SET NORTHPOLE TO LATLNG( 90, 0). //Reference heading
SET AG9 TO FALSE.
SET lockcounter TO 0.
SET nextWaypointHeading TO 0.
SET spc TO "      ".

SET overSpeedDownSlopeBrakeTime TO 0.3.
SET extremeSlopeAngle TO 8.
SET overSpeedCruise TO 8.
SET extraBrakeTime TO 0.7.
SET cruiseSpeedBrakeTime TO 1.
SET currentSlopeAngle TO 0.
SET brakesOn TO TRUE.
SET lastBrake TO -1.
SET lastEvent TO -1.
SET brakeUseCount TO 0.
SET currentOverSpeed TO overSpeedCruise.
SET currentBrakeTime TO cruiseSpeedBrakeTime.
SET stopDistance TO 0.5.
SET gradient TO 0.
SET wpm TO LIST().
SET navpoints TO LIST().
SET contractWayPoints TO LIST().
SET sp TO LIST().

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
SET route TO LIST().
SET rwaypoint TO -1.

ON AG1 {
  if AG1 {
    if route:LENGTH = 0 {
      SET __goal TO __current.
      nav_marker().
    }
  } else {
    CLEARVECDRAWS().
  }
  PRESERVE.
}

display_HUD().
until runmode = -1 {
  //Update the compass:
  // I want the heading TO match the navball
  // and be out of 360' instead of +/-180'
  // I do this by judging the heading relative
  // TO a lAT/LNG SET TO the north pole
  IF northPole:bearing <= 0 {
    SET cHeading TO ABS(northPole:bearing).
  } ELSE {
    SET cHeading TO (180 - northPole:bearing) + 180.
  }
  SET targetspeed TO targetspeed + 0.1 * SHIP:CONTROL:PILOTWHEELTHROTTLE.
  SET targetspeed TO max(-1, targetspeed).
  SET upvec TO up:vector.
  SET velvec TO ship:velocity:surface:normalized.
  SET dp TO vdot(velvec,upvec).
  SET currentSlopeAngle TO 90 - arccos(dp).

  SET facvec TO SHIP:FACING.

  IF ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCCONNECTION(SHIP) = FALSE AND runmode <> 12 {
    SET runmode TO 12.
    set_speed(0).
    BRAKES ON.
    SET brakesOn TO TRUE.
  } else if ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCCONNECTION(SHIP) AND runmode = 12 {
    SET runmode TO 0.
    if route:LENGTH <> 0 {
      restore_speed().
      SET brakesOn TO FALSE.
      SET WARP TO 0.
    }
    SET lastEvent TO TIME:SECONDS.
  }

  LOCAL predicted1 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5))).
  LOCAL predicted2 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5)+1)).
  LOCAL heightdiff IS predicted2:TERRAINHEIGHT - predicted1:TERRAINHEIGHT.
  LOCAL distance IS (predicted1:POSITION - predicted2:POSITION):MAG.
  SET angle TO ARCSIN(heightdiff/distance).
  SET gradient TO ROUND(TAN(currentSlopeAngle),5).
  SET pangle TO MAX(currentSlopeAngle,angle) - MIN(currentSlopeAngle,angle).

  SET stopDistance TO get_stop_distance(GROUNDSPEED+0.5).

// LOG currentSlopeAngle+","+gradient+","+(1/(const_gravity + gradient)) TO "0:/debug.log".
  if ABS(GROUNDSPEED) > 0.1 {
    SET slopeSpeed TO SQRT(get_stop_distance(settings["DefaultSpeed"])*(( 2 /const_gravity / (1 / const_gravity + gradient)))).   // √(30×(2÷1.62÷(1÷(1.62−0.823))))
  }
  if route:LENGTH <> 0 AND rwaypoint <> -1  AND rwaypoint < route:LENGTH-1 {
    // IF runmode = 0 { //Govern the rover

    SET headingDifference TO route[rwaypoint]:HEADING - cHeading.
    SET nextWaypointHeading TO route[rwaypoint+1]:HEADING - cHeading.

    IF ROUND(GROUNDSPEED) = 0 AND abs(targetspeed) > 0 {
      SET runmode TO 5.
      SET lastEvent TO TIME:SECONDS.
    } else {
      if pangle > 5 {          //
        SET runmode TO 2.
        set_speed(1).
      } else if angle < settings["MinSlope"] AND runmode <> 4 {
        LOCK targetspeed TO slopeSpeed.
        // set_speed(3).
        SET runmode TO 4.
      } else if ABS(headingDifference) > 40 AND targetspeed = lastTargetSpeed AND runmode <> 2 AND runmode <> 4 {
        set_speed(1).
        SET runmode TO 3.
      }
      if __grid:DISTANCE < MAX(25,(stopDistance*1.5)) AND ABS(headingDifference > 40) and rwaypoint <> 0 {
        SET runmode TO 1.
        if ABS(targetHeading - cHeading) > 40 {
          set_speed(1).
        } else {
          set_speed(3).
        }
      }
      if route[rwaypoint]:DISTANCE < MAX(15,stopDistance) {
        next_waypoint(3).
      }
    }
    if runmode = 2 AND pangle < 5 {
      SET runmode TO 0.
      restore_speed().
    } else if runmode = 3 {
      LOCAL waypointcounter IS rwaypoint-1.
      if rwaypoint = 0 {
        SET waypointcounter TO rwaypoint.
      }
      if ABS(headingDifference) < 10 {
        restore_speed().
        SET runmode TO 0.
      }
    } else if runmode = 4 AND angle > settings["MinSlope"] {
      UNLOCK targetspeed.
      restore_speed().
    } else if runmode = 5 AND ABS(GROUNDSPEED) > 0 {
      restore_speed().
      SET runmode TO 0.
    } else if runmode = 5 AND ROUND(GROUNDSPEED) = 0 AND (TIME:SECONDS - lastEvent) > 5 {
      SET targetspeed TO -1.
      LOCK targetHeading TO (__grid:HEADING - 90).
    } else if runmode = 5 AND ABS(GROUNDSPEED) > 0 AND (TIME:SECONDS - lastEvent) > 10 {
      SET targetspeed TO 0.
      SET runmode to 6.
    } else if runmode = 6 AND round(GROUNDSPEED) = 0 AND (TIME:SECONDS - lastEvent) > 10 {
      set_speed(2).
      SET lastEvent TO TIME:SECONDS.
    } else if runmode = 6 AND abs(GROUNDSPEED) > 0 AND (TIME:SECONDS - lastEvent) > 10 {
      LOCK targetHeading TO __grid:HEADING.
      SET runmode TO 2.
    }
  } else {
    IF navpoints:LENGTH <> 0 AND rwaypoint <> -1 {
      SET route TO LIST().
      SET targetspeed TO 0.
      SET rwaypoint TO -1.
      SET runmode TO 0.
      LOCAL gl IS navpoints[0].
      navpoints:REMOVE(0).
      RUNPATH("/astar-rover/astar","LATLNG",gl,false).
      start_navigation().
    } else {
      if AG1 {
        PRINT "---{   Rover has arrived at location  }---         " + spc AT (2,1).
        SET targetspeed TO 0.
        BRAKES ON.
        UNLOCK targetheading.
      } else {
        PRINT"---{   Manual controle   }---" + spc AT (2,1).
      }
    }
  }


  IF targetspeed <> 0 { //IF we should be going forward
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
  } ELSE { // IF value is out of range or zero, stop.
    SET wtVAL TO 0.
    brakes on.
  }

  IF AG1 { //Activate autopilot if Action group 1 is on
    SET errorSteering TO (targetheading - cHeading).
    IF errorSteering > 180 { //Make sure the headings make sense
      SET errorSteering TO errorSteering - 360.
    } ELSE IF errorSteering < -180 {
      SET errorSteering TO errorSteering + 360.
    }
    SET desiredSteering TO -errorSteering / 20.
    SET kturn TO min( 1, max( -1, desiredSteering)) * turnlimit.
  } ELSE {
    SET kturn TO turnlimit * SHIP:CONTROL:PILOTWHEELSTEER.
  }

  if GROUNDSPEED < 5 AND targetspeed> 0 {
     //Safety adjustment to help reduce roll-back at low speeds
     set wtVAL to min( 1, max( -0.2, wtVAL)).
  }


  when abs(GROUNDSPEED) > ABS(targetspeed) then {  // borrowed from gaiiden / RoverDriver https://github.com/Gaiiden/RoverDriver/blob/master/begindrive.txt
    if brakesOn = false {
      SET brakesOn TO true.
      brakes on.
      SET brakeUseCount TO brakeUseCount + 1.

      if abs(currentSlopeAngle) > extremeSlopeAngle {
        SET currentBrakeTime TO overSpeedDownSlopeBrakeTime + extraBrakeTime.
      }

      SET lastBrake TO time:seconds.
    }.
    preserve.
  }.

  // do we need to disable the brakes?
  when brakesOn = true and (abs(GROUNDSPEED) <= ABS(targetspeed) OR time:seconds - lastBrake >= currentBrakeTime) then {
    brakes off.
    SET brakesOn TO false.
    preserve.
  }.


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
        RUNPATH("/astar-rover/astar","LATLNG",__goal,false).
      } else {
        RUNPATH("/astar-rover/astar","LATLNG",navpoints[0],false).
        navpoints:REMOVE(0).
      }
      start_navigation().
      PRINT "  ---{   Navigating to LAT " + round(__goal:LAT,1) +" : LNG " + round(__goal:LNG,1) +  "   }---" AT (0,1).
    }
    ELSE IF K = TERMINAL:INPUT:PAGEUPCURSOR {
      SET targetspeed TO targetspeed + 0.5.
      update_setting("DefaultSpeed",targetspeed).
      SET lasttargetspeed TO targetspeed.
      }
    ELSE if K = TERMINAL:INPUT:PAGEDOWNCURSOR {
      SET targetspeed TO targetspeed - 0.5.
      update_setting("DefaultSpeed",targetspeed).
      SET lastTargetSpeed TO targetspeed.
    }
    ELSE IF K = TERMINAL:INPUT:HOMECURSOR {
      if runmode = 11 {
        SET runmode TO 0.
        display_HUD().
      } else {
        SET __goal TO SHIP:GEOPOSITION.
        nav_marker().
        SET targetspeed TO 0.
        SET route TO LIST().
        SET rwaypoint TO -1.
      }
    }
    ELSE IF K:TOUPPER = "I" {
      navpoints:ADD(__goal).
      waypoint_marker().
    }
    ELSE IF K:TOUPPER = "W" {
      CLEARSCREEN.
      LOCAL WPS IS ALLWAYPOINTS().
      SET contractWayPoints TO LIST().
      SET runmode TO 10.
      FOR WP IN WPS {
        IF WP:BODY:NAME = BODY:NAME {
          contractWayPoints:ADD(WP).
        }
      }
    }
    ELSE IF K:TOUPPER = "N" {
      next_waypoint(2).
    }
    ELSE IF K:TOUPPER = "C" {
      runpath("0:/astar-rover/setup").
      REBOOT.
    }
    ELSE IF K:TOUPPER = "S" {
      SET runmode TO 11.
      SET sp TO get_science_parts().
      science_menu().
    }
    ELSE IF K:TOUPPER = "R" {
      display_HUD.
    }
    ELSE IF K = TERMINAL:INPUT:ENDCURSOR {
      SET runmode TO -1.
      CLEARVECDRAWS().
      SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    } ELSE IF N <> -99 {
      if runmode = 11 {
        do_science(N).
        display_HUD().
        SET runmode TO 0.
      } else if runmode = 10 {
        if N <= contractWayPoints:LENGTH {
          SET runmode TO 0.
          LOCAL w IS contractWayPoints[N-1].
          RUNPATH("/astar-rover/astar","WAYPOINT",w:NAME,false).
          if route:LENGTH <> 0 {
            SET __goal TO LATLNG(route[route:LENGTH-1]:LAT+0.1,route[route:LENGTH-1]:LNG).
            start_navigation().
            PRINT "  ---{   Navigating to " + w:NAME +"   }---" AT (0,1).
            CLEARVECDRAWS().
          } else {
            display_HUD().
          }
        }
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
      PRINT "(" + y + ") " + c:NAME AT (4,y+4).
      LOCAL p IS c:GEOPOSITION.
      PRINT " " + round((p:DISTANCE/1000),2) + " km  " AT (30,y+4).
      SET y TO y+1.
    }
  } else if runmode <> 11 {
    PRINT ROUND( targetspeed, 1) + spc AT (20, 6).
    PRINT ROUND( GROUNDSPEED) + spc AT (20, 7).
    PRINT round(__goal:DISTANCE) + spc AT (20, 8).

    PRINT ROUND( targetheading, 2) + spc AT (20, 10).
    PRINT ROUND( cheading, 2) + spc AT (20, 11).

    IF DEFINED route AND route:LENGTH <> 0 AND rwaypoint <> route:LENGTH-1 AND rwaypoint <> -1 {
      PRINT round(ABS(route[rwaypoint+1]:HEADING)) + spc AT (20, 12).
      PRINT round(nextWaypointHeading) + spc AT (20,13).
    }
    PRINT ROUND(__grid:DISTANCE, 2) + spc AT (20, 15).
    PRINT route:LENGTH + spc AT (20, 16).
    PRINT rwaypoint + spc AT (20, 17).

    PRINT round(currentSlopeAngle,2) + spc AT (20,20).
    PRINT round(angle,2) + spc AT (20,21).
    PRINT round(pangle,2) + spc AT (20,22).

    PRINT round(stopDistance,4) + spc AT (20,24).
    PRINT round(gradient,4) + spc AT (20,25).

    PRINT Runmode + spc AT (20, 27).
    PRINT AG1 + "   " AT (20, 28).

    PRINT ROUND( wtVAL, 2) + spc AT (20, 30).
    PRINT ROUND( kTurn, 2) + spc AT (20, 31).

    PRINT ROUND(eWheelThrottle,2)  AT ( 6, 33).
    PRINT ROUND(iWheelThrottle,2) AT (16,33).
  }
  SET looptime TO TIME:SECONDS - loopEndTime.
  SET loopEndTime TO TIME:SECONDS.
  WAIT 0. //  We only need to run an iteration once per physics tick.  Ensure that we pause until the next tick.
}

    FUNCTION display_HUD {
      CLEARSCREEN.
      PRINT "Target Speed    :" AT (2,6).
      PRINT "Surface Speed   :" AT (2,7).
      PRINT "Distance to goal:" AT (2,8).

      PRINT "Target Heading  :" AT (2,10).
      PRINT "CurrentHeading  :" AT (2,11).
      PRINT "  Next Heading  :" AT (2,12).
      PRINT "  Next Bearing  :" AT (2,13).

      PRINT "Waypoint Dist   :"  AT (2,15).
      PRINT "Waypoints       :" AT (2,16).
      PRINT "Current WP      :" AT (2,17).

      PRINT "Current Angle   :" AT (2,20).
      PRINT "Predicted Angle :" AT (2,21).

      PRINT "Difference      :" AT (2,22).
      PRINT "Stopping Dist   :" AT (2,24).
      PRINT "Gradient        :" AT (2,25).

      PRINT "Runmode         :" AT (2,27).
      PRINT "Cruise Control  :" AT (2,28).

      PRINT "Commanded tVAL  :" AT (2,30).
      PRINT "Commanded Turn  :" AT (2,31).

      PRINT "E:" AT ( 2, 33).
      PRINT "I:" AT (12,33).
    }

    FUNCTION nav_marker {
      SET vg TO VECDRAWARGS(
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
      SET rwaypoint TO 1.
      if route:LENGTH <> 0 {
        SET __grid TO LATLNG(route[rwaypoint]:LAT,route[rwaypoint]:LNG).
        LOCK targetHeading TO __grid:HEADING.
        BRAKES OFF.
        LIGHTS ON.
        SET AG1 TO TRUE.
        restore_speed().
        SET lastEvent TO TIME:SECONDS.
        CLEARVECDRAWS().
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
      SET targetspeed TO settings["DefaultSpeed"].
    }

    FUNCTION get_stop_distance{
      PARAMETER speed.
      return speed^2 / ( 2 * const_gravity * ( 1 / const_gravity + gradient)).
    }

    FUNCTION next_waypoint
    {
      PARAMETER mode.
      SET rwaypoint TO rwaypoint + 1.
      if rwaypoint < route:LENGTH {
        SET __grid TO LATLNG(route[rwaypoint]:LAT,route[rwaypoint]:LNG).
        LOCK targetHeading TO __grid:HEADING.
        SET runmode TO mode.
      } else {
        SET rwaypoint TO rwaypoint - 1.
      }
    }
    FUNCTION get_science_parts {
      return SHIP:PARTSTAGGED("Skience").

    }

    FUNCTION update_setting{
      PARAMETER key,value.
      SET settings[key] TO value.
      WRITEJSON(settings,"1:/config/settings.json").
    }

    FUNCTION science_menu {
      CLEARSCREEN.
      PRINT "  ---{  Select a science part }---" AT (2,2).
      LOCAL y IS 1.
      FOR p IN sp {
        PRINT "(" + y + ") " + p:TITLE AT (4,y+4).
        SET y TO y + 1.
      }
    }

    FUNCTION do_science {
      PARAMETER N.
      LOCAL sc TO sp[N-1].
      LOCAL mods IS sc:MODULES.
      FOR mo IN mods {
        if mo = "ModuleScienceExperiment"{
          transmit_science(sc:GETMODULE("ModuleScienceExperiment")).
        } else if mo = "DMModuleScienceAnimate" {
          transmit_science(sc:GETMODULE("DMModuleScienceAnimate")).
        }
      }
    }

    FUNCTION transmit_science {
      PARAMETER M.

      PRINT "Preparing science part" AT (2,10).
      M:DUMP.
      M:RESET.
      PRINT "Performing Science" AT (2,11).
      WAIT 0.
      M:DEPLOY.

      PRINT "Transmitting Science" AT (2,12).
      WAIT UNTIL M:HASDATA.
      M:TRANSMIT.
      M:RESET.
    }
