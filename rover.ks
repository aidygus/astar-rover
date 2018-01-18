// rover.ks
// Original direction and speed routines written by KK4TEE
// Updated with waypoint management by aidygus
// License: GPLv3
//
// ThiS program provides waypoint functionality
// using an astar based algorythm to calculate
// safest possible route from start to goal
// and monitoring for fully automated rovers

PARAMETER debug IS true.

SET vol TO "0:".
IF EXISTS("1:/astar-rover/libs.km") {
  SET vol TO "1:".
}

if EXISTS("1:/config/settings.json") = FALSE {
  runpath("0:/astar-rover/setup").
  REBOOT.
} else {
  SET settings TO READJSON("1:/config/settings.json").
  COPYPATH("1:/config/settings.json","0:/astar-rover/backup/"+SHIP:NAME+".json").
}

RUNPATH(vol+"/astar-rover/libs").
SET const_gravity TO BODY:mu / BODY:RADIUS ^ 2.
Wait 0.01.
lock turnlimit to min(1, settings["TurnLimit"] / SHIP:GROUNDSPEED). //Scale the
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
SET slopeSpeed TO settings["DefaultSpeed"].
SET targetHeading TO 90. //Used for autopilot steering
SET NORTHPOLE TO LATLNG( 90, 0). //Reference heading
SET AG9 TO FALSE.
SET AG2 TO TRUE.
SET lockcounter TO 0.
SET nextWaypointHeading TO 0.
SET spc TO "     ".
SET header TO "".
SET waitCursor TO LIST("-","\","|","/").
SET waitCount TO 0.
SET coList TO LEXICON("Kerbin",1.1108,"Mun",0.60,"Minmus",0.65,"Duna",0.4,"Eve",0.3).  //  Need an equation to work out what friction coefficient is for given gravity
SET KUNIVERSE:DEFAULTLOADDISTANCE:LANDED:UNLOAD TO 30000.
SET KUNIVERSE:DEFAULTLOADDISTANCE:LANDED:LOAD TO 29500.
WAIT 0.001. // See paragraph above: "wait between load and pack changes"
SET KUNIVERSE:DEFAULTLOADDISTANCE:LANDED:PACK TO 39999.
SET KUNIVERSE:DEFAULTLOADDISTANCE:LANDED:UNPACK TO 29000.
wait 0.001.
SET DirectionOT TO 1.
SET page TO 0.

SET overSpeedDownSlopeBrakeTime TO 0.3.
SET extremeSlopeAngle TO 8.
SET overSpeedCruise TO 8.
SET extraBrakeTime TO 0.7.
SET cruiseSpeedBrakeTime TO 1.
SET currentSlopeAngle TO 0.
SET brakesOn TO TRUE.
SET lastBrake TO -1.
SET lastEvent TO LIST(TIME:SECONDS,0,0,0,0,0,0,0).
SET lastBat TO -1.
SET brakeUseCount TO 0.
SET currentOverSpeed TO overSpeedCruise.
SET currentBrakeTime TO cruiseSpeedBrakeTime.
SET stopDistance TO 0.5.
SET gradient TO 0.
SET wpm TO LIST().
SET navpoints TO LIST().
SET contractWayPoints TO LIST().
SET maxCharge TO STAGE:ELECTRICCHARGE.
LIST RESOURCES IN reslist.
FOR res IN reslist{
  if res:NAME = "ElectricCharge" {
    SET Electric TO res.
  }
}
LOCK chargeLevel TO ROUND(100*Electric:AMOUNT/Electric:CAPACITY).
SET lastCharge TO Electric:AMOUNT.
SET menu TO 0.
SET nextWrite TO -1.
//     0 Main Menu
//     1 Select Waypoint
//     2 Do Science
//     3 No Connection
//     4 Low Power

clearscreen.
CLEARVECDRAWS().
sas off.
rcs off.
BRAKES ON.
LOCK throttle TO 0.

SET runmode TO 0.
    // Valid runmodes
    //    0  Normal Operation
    //    1  Approaching Waymarker
    //    2  Change of slope angle ahead
    //    3  Moving away from waypoint
    //    4  Negotiating slope exceeding MinSlope (Depricated)
    //    5  Has hit an obstacle.
    //    6  Attempting to move round obstacle.
    //    7  Manual Hold

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
      center(header,1).
    }
  } else {
    CLEARVECDRAWS().
    SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    center(spc + spc + spc + "---{   Manual control   }---" + spc + spc + spc,1).
  }
  PRESERVE.
}

display_HUD().
play_sounds("start").
until runmode = -1 {
  IF chargeLevel < 10 AND runmode <> 13
  {
    LIGHTS OFF.
    CLEARSCREEN.
    hold_poition(13).
    SET menu TO 4.
  }
  else if chargeLevel > 30 AND runmode = 13
  {
    restore_operations().
    SET menu TO 0.
  }
  ELSE IF (ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCCONNECTION(SHIP) = FALSE) AND runmode <> 12
  {
    CLEARSCREEN.
    SET menu TO 3.
    hold_poition(12).
  }
  else if (ADDONS:RT:AVAILABLE AND ADDONS:RT:HASKSCCONNECTION(SHIP)) AND runmode = 12
  {
    restore_operations().
    SET menu TO 0.
  }

  if runmode <> 13 AND runmode <> 12
  {

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
    SET upvec TO up:vector.
    SET velvec TO ship:velocity:surface:normalized.
    SET dp TO vdot(velvec,upvec).
    SET dir TO vdot(SHIP:FACING:VECTOR,SRFPROGRADE:VECTOR).
    SET currentSlopeAngle TO 90 - arccos(dp).

    SET facvec TO SHIP:FACING.
    SET targetspeed TO targetspeed + 0.1 * SHIP:CONTROL:PILOTWHEELTHROTTLE.
    SET targetspeed TO max(0, targetspeed).

    LOCAL predicted1 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5))).
    LOCAL predicted2 IS body:GEOPOSITIONOF(facvec + V(0,0,MAX(15,stopDistance+5)+1)).
    LOCAL heightdiff IS predicted2:TERRAINHEIGHT - predicted1:TERRAINHEIGHT.
    LOCAL distance IS (predicted1:POSITION - predicted2:POSITION):MAG.
    LOCAL playsound IS "blip".
    SET angle TO ARCSIN(heightdiff/distance).
    SET gradient TO ROUND(TAN(currentSlopeAngle),5).
    SET pangle TO MAX(currentSlopeAngle,angle) - MIN(currentSlopeAngle,angle).

    SET stopDistance TO get_stop_distance(GROUNDSPEED+0.5).


    if route:LENGTH <> 0 AND rwaypoint <> -1  AND rwaypoint <= route:LENGTH-1
    {
      // IF runmode = 0 { //Govern the rover

      SET headingDifference TO route[rwaypoint]:HEADING - cHeading.
      if rwaypoint+1 <> route:LENGTH {
        SET nextWaypointHeading TO route[rwaypoint+1]:HEADING - cHeading.
        IF nextWaypointHeading > 180 { //Make sure the headings make sense
          SET nextWaypointHeading TO nextWaypointHeading - 360.
        } ELSE IF nextWaypointHeading < -180 {
          SET nextWaypointHeading TO nextWaypointHeading + 360.
        }
      }

      if runmode <> 7 {
        IF navpoints:LENGTH <> 0 AND rwaypoint = route:LENGTH-1 AND route[rwaypoint]:DISTANCE < 95 AND (TIME:SECONDS - lastEvent[0]) > 15 {
          SET route TO LIST().
          SET targetspeed TO 0.
          SET rwaypoint TO -1.
          SET runmode TO 0.
          LOCAL gl IS navpoints[0].
          navpoints:REMOVE(0).
          RUNPATH("0:/astar-rover/astar","LATLNG",gl,false).
          start_navigation().
        }
        IF ROUND(GROUNDSPEED,1) = 0.0 AND abs(targetspeed) > 0  AND TIME:SECONDS - lastEvent[0] > 10 {
          SET runmode TO 5.
          SET lastEvent[5] TO TIME:SECONDS.
        } else {
          if ABS(headingDifference) > 40 AND runmode <> 5 AND runmode <> 6 AND runmode <> 3 {
            play_sounds("direction").
            set_speed(MAX(1,settings["DefaultSpeed"]/6),3).
          }
          if __grid:DISTANCE < 5 + stopDistance AND ABS(nextWaypointHeading) > 40 AND runmode <> 1 {
            set_speed(MAX(1,settings["DefaultSpeed"]/4),1).
          }
          if ABS(pangle) > 5 AND runmode <> 2 AND GROUNDSPEED > 2 {  // Predicted slope change angle
            set_speed(MAX(1,settings["DefaultSpeed"]/4),2).
            play_sounds("slopealert").
          }
        }
        if runmode <> 0 {
          if runmode = 2 AND ABS(pangle) < 5 {
            SET runmode TO 0.
            restore_speed().
          } else if runmode = 3 {
            if ABS(headingDifference) < 6 {
              restore_speed().
              SET runmode TO 0.
            }
          } else if runmode = 5 AND ROUND(GROUNDSPEED,1) = 0.0 AND (TIME:SECONDS - lastEvent[5]) > 10 {
            play_sounds("alert").
            SET targetspeed TO MAX(1,settings["DefaultSpeed"]/4).
            SET DirectionOT TO -1.
            LOCK targetHeading TO (__grid:HEADING - 90).
            SET lastEvent[5] TO TIME:SECONDS.
          } else if runmode = 5 AND ROUND(GROUNDSPEED,1) > 0.0 AND (TIME:SECONDS - lastEvent[5]) > 10 {
            set_speed(0,6).
            SET DirectionOT TO 1.
          } else if runmode = 6 AND round(GROUNDSPEED,1) = 0.0 AND (TIME:SECONDS - lastEvent[6]) > 10 {
            set_speed(MAX(1,settings["DefaultSpeed"]/4),6).
          } else if runmode = 6 AND ROUND(GROUNDSPEED,1) > 0.0 AND (TIME:SECONDS - lastEvent[6]) > 15 {
            LOCK targetHeading TO __grid:HEADING.
            SET runmode TO 3.
          }
        }
        if __grid:DISTANCE < MAX(30,stopDistance) {
          next_waypoint(3).
        }

        if angle < -4 AND runmode = 0 {
          SET targetspeed TO get_slope_speed().
        }
      }
      if rwaypoint <> -1 AND rwaypoint = (route:LENGTH-1) AND route[rwaypoint]:DISTANCE < 50 {
        SET header TO "---{   Rover has arrived at location  }---         " + spc.
        PRINT header AT (2,1).
        hold_poition(7).
        SET lastEvent[0] TO TIME:SECONDS.
      }

      IF targetspeed = 0 AND ROUND(GROUNDSPEED,1) = 0 {
        SET wtVAL TO 0.
        brakes on.
      } else {
        if brakesOn = false {
          brakes off.
        }
        if targetspeed > settings["DefaultSpeed"] {
          SET targetspeed TO settings["DefaultSpeed"].
        }
        if Runmode <> 7 {
          SET eWheelThrottle TO targetspeed - GROUNDSPEED.
          SET iWheelThrottle TO min( 1, max( -1, iWheelThrottle +
                                              (looptime * eWheelThrottle))).
          SET wtVAL TO eWheelThrottle + iWheelThrottle * DirectionOT.//PI controler

          // Stop rollback at slow speeds
          if ABS(GROUNDSPEED) < 2 and runmode <> 0 AND wtVAL < 0.2 {
             set wtVAL to min( 0.2, max( -0.2, wtVAL)).
          }

          // If rover is rolling back then invert wtVAL to make it go forwards
          if targetspeed > 0 AND dir < 0 AND wtVAL < 0 AND DirectionOT = 1 {
            SET wtVAL TO -wtVAL.
          }

          // Slow down rover accelleration to stop it from wheelying in low gravity
          if GROUNDSPEED < ABS(targetspeed)*0.5 OR GROUNDSPEED <= settings["DefaultSpeed"]/3 {
            SET wtVAL TO MIN(wtVAL, MAX(0.5,0.5*GROUNDSPEED)).
          }
          if ABS(currentSlopeAngle) > 10 AND GROUNDSPEED < settings["DefaultSpeed"]/3 {
            SET wtVAL TO wtVAL + (0.02 * ABS(currentSlopeAngle)).
          }
        } else {
          SET wtVAL TO 0.
        }
      }
    }

    IF AG1 AND runmode <> 7 { //Activate autopilot if Action group 1 is on
      SET errorSteering TO (targetheading - cHeading).
      IF errorSteering > 180 { //Make sure the headings make sense
        SET errorSteering TO errorSteering - 360.
      } ELSE IF errorSteering < -180 {
        SET errorSteering TO errorSteering + 360.
      }
      SET desiredSteering TO -errorSteering / 20.
      if ABS(GROUNDSPEED) < MAX(2,settings["DefaultSpeed"]/4) {
        SET kturn TO desiredSteering.
      } else {
        SET kturn TO min( 1, max( -1, desiredSteering )) * turnlimit.
      }
    } ELSE {
      SET kturn TO turnlimit * SHIP:CONTROL:PILOTWHEELSTEER.
    }

    when abs(GROUNDSPEED) > ABS(targetspeed*1.1) then {  // borrowed from gaiiden / RoverDriver https://github.com/Gaiiden/RoverDriver/blob/master/begindrive.txt
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
    when brakesOn = true and (abs(GROUNDSPEED) <= ABS(targetspeed) OR time:seconds - lastBrake >= currentBrakeTime) AND targetspeed <> 0 then {
      brakes off.
      SET brakesOn TO false.
      preserve.
    }.

  if wpm:LENGTH <> 0 {
    nav_lines().
  }
  //Handle User Input using action groups
  IF TERMINAL:INPUT:HASCHAR
  {
    LOCAL playsound IS "blip".
    LOCAL K IS TERMINAL:INPUT:GETCHAR().
    if menu = 5 {
      SET TERMINAL:WIDTH TO 50.
      SET TERMINAL:HEIGHT TO 40.
      SET TERMINAL:CHARHEIGHT TO 8.
      SET menu TO 0.
      display_HUD().
    }
    LOCAL N IS K:TONUMBER(-99).
    IF K = TERMINAL:INPUT:UPCURSORONE
    {
      SET __goal TO LATLNG(__goal:LAT+0.01,__goal:LNG).
      nav_marker().
    }
    ELSE IF K = TERMINAL:INPUT:DOWNCURSORONE
    {
      SET __goal TO LATLNG(__goal:LAT-0.01,__goal:LNG).
      nav_marker().
    }
    ELSE IF K = TERMINAL:INPUT:LEFTCURSORONE
    {
      if menu = 1 {
        SET page TO page - 1.
        CLEARSCREEN.
      } else {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG-0.01).
        nav_marker().
      }
    }
    ELSE IF K = TERMINAL:INPUT:RIGHTCURSORONE
    {
      if menu = 1 {
        SET page TO page + 1.
        CLEARSCREEN.
      } else {
        SET __goal TO LATLNG(__goal:LAT,__goal:LNG+0.01).
        nav_marker().
      }
    }
    ELSE IF K = TERMINAL:INPUT:RETURN
    {
      if navpoints:LENGTH = 0 {
        RUNPATH("0:/astar-rover/astar","LATLNG",__goal,false).
      } else {
        RUNPATH("0:/astar-rover/astar","LATLNG",navpoints[0],false).
        navpoints:REMOVE(0).
      }
      start_navigation().
      SET header TO "---{   Navigating to LAT " + round(__goal:LAT,1) +" : LNG " + round(__goal:LNG,1) +  "   }---" + spc + spc.
      PRINT header AT (2,1).
    }
    ELSE IF K = TERMINAL:INPUT:PAGEUPCURSOR
    {
      SET settings["DefaultSpeed"] TO settings["DefaultSpeed"] + 0.5.
      SET targetspeed TO settings["DefaultSpeed"].
      update_setting("DefaultSpeed",targetspeed).
      }
    ELSE if K = TERMINAL:INPUT:PAGEDOWNCURSOR
    {
      SET settings["DefaultSpeed"] TO settings["DefaultSpeed"] - 0.5.
      SET targetspeed TO settings["DefaultSpeed"].
      update_setting("DefaultSpeed",targetspeed).
    }
    ELSE IF K = TERMINAL:INPUT:HOMECURSOR {
      if menu <> 0 {
        SET runmode TO 0.
        display_HUD().
        SET menu TO 0.
      } else {
        SET __goal TO SHIP:GEOPOSITION.
        nav_marker().
        SET targetspeed TO 0.
        SET route TO LIST().
        SET rwaypoint TO -1.
        hold_poition(0).
      }
    }
    ELSE IF K = TERMINAL:INPUT:ENDCURSOR {
      SET runmode TO -1.
      CLEARVECDRAWS().
      SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
    }
    ELSE IF K:TOUPPER = "I" {
      navpoints:ADD(__goal).
      waypoint_marker().
    }
    ELSE IF K:TOUPPER = "W" {
      CLEARSCREEN.
      SET menu TO 1.
      SET page TO 0.
      LOCAL WPS IS ALLWAYPOINTS().
      SET contractWayPoints TO LIST().
      // SET runmode TO 10.
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
      SET settings TO READJSON("1:/config/settings.json").
      display_HUD().
    }
    ELSE IF K:TOUPPER = "S" {
      SET menu TO 2.
      science_menu().
    }
    ELSE IF K:TOUPPER = "R" {
      SET menu TO 0.
      display_HUD().
    }
    ELSE IF K:TOUPPER = "H" {
      if runmode = 7 {
        SET runmode to 0.
        restore_speed().
      } else {
        BRAKES ON.
        SET targetspeed TO 0.
        SET brakesOn TO TRUE.
        SET runmode TO 7.
      }
    }
    ELSE IF K:TOUPPER = "V" {
      CLEARVECDRAWS().
    }
    ELSE IF K:TOUPPER = "M" {
      load_map().
    }
    ELSE IF K:TOUPPER = "L" {
      if wpm:LENGTH <> 0 {
        CLEARVECDRAWS().
        SET wpm TO LIST().
      } else {
        nav_lines().
      }
    }
    ELSE IF K = "," {
      SET KUNIVERSE:TIMEWARP:MODE TO "PHYSICS".
      SET WARP TO 0.
    }
    ELSE IF K = "." {
        SET KUNIVERSE:TIMEWARP:MODE TO "PHYSICS".
        SET WARP TO 1.
    }
    ELSE IF N <> -99 {
      if menu = 2 {
        do_science(N).
        display_HUD().
        SET menu TO 0.
      } else if menu = 1 {
        if N = 0 SET N TO 10.
        SET N TO N + (page*10).
        if N <= contractWayPoints:LENGTH {
          LOCAL w IS contractWayPoints[N-1].
          RUNPATH("0:/astar-rover/astar","WAYPOINT",w:NAME,false).
          SET menu TO 0.
          if route:LENGTH <> 0 {
            SET __goal TO LATLNG(route[route:LENGTH-1]:LAT,route[route:LENGTH-1]:LNG).
            start_navigation().
            SET header TO "  ---{   Navigating to " + w:NAME +"   }---".
            PRINT header AT (0,1).
            CLEARVECDRAWS().
          } else {
            display_HUD().
          }
        }
      }
      PRINT spc + spc + spc + spc AT (2,TERMINAL:HEIGHT - 2).
    } ELSE {
      SET playsound TO "error".
      PRINT "# UNKNOWN COMMAND # " AT (2,TERMINAL:HEIGHT - 2).
    }
    play_sounds(playsound).
  }

  IF targetHeading > 360 {
    SET targetHeading TO targetHeading - 360.
  }
  ELSE IF targetHeading < 0 {
    SET targetHeading TO targetHeading + 360.
  }
}
else
{
  SET wtVAL TO 0.
  SET kTurn TO 0.
  BRAKES ON.
}

  SET SHIP:CONTROL:WHEELTHROTTLE TO WTVAL.
  SET SHIP:CONTROL:WHEELSTEER TO kTurn.

  if menu = 1 {
    PRINT "Press a number to select a waypoint" AT (2, 3).
    if page < 0 SET page TO FLOOR(contractWayPoints:LENGTH/10).
    if page > FLOOR(contractWayPoints:LENGTH/10) SET page TO 0.
    PRINT "Page " + (page+1) + " of " + (CEILING(contractWayPoints:LENGTH/10)) AT (4,20).
    LOCAL y IS 1.
    UNTIL y = 11 OR (y+(page*10)-1) >= contractWayPoints:LENGTH{
      SET c TO contractWayPoints[y+(page*10)-1].
      PRINT "(" + y + ") " + c:NAME AT (4,y+4).
      LOCAL p IS c:GEOPOSITION.
      PRINT " " + round((p:DISTANCE/1000),2) + " km  " AT (30,y+4).
      SET y TO y+1.
      if y = 11 break.
    }
    WAIT 0.1.
  }
  else if menu = 3
  {
    CLEARSCREEN.
    center("---{    CONNECTION LOST    }---",5).

    if TIME:SECONDS - lastEvent[0] > 0.3 {
      PRINT "Acquiring Signal " + (waitCursor[waitCount]) + spc AT (2,10).
      SET waitCount TO waitCount + 1.
      if waitCount = 4 {
        SET waitCount TO 0.
      }
      SET lastEvent[0] TO TIME:SECONDS.
    }
  }
  else if menu = 4
  {
    CLEARSCREEN.
    center("---{    LOW POWER MODE    }---",5).
    center(spc + "Remaining charge : " + ROUND( chargeLevel, 1) + "%" + spc,8).
    display_battery(11,chargeLevel).
  }
  else if menu = 0 {
    PRINT ": " +  ROUND( targetspeed, 1) + " m/s" + spc AT (18, 6).
    PRINT ": " +  ROUND( GROUNDSPEED,1) + " m/s" + spc AT (18, 7).
    PRINT ": " + ROUND ((GROUNDSPEED * 18)/5) + " km/h" + spc AT (30,7).
    PRINT ": " +  round(__goal:DISTANCE/1000,2) + " km" + spc AT (18, 8).

    PRINT ": " +  ROUND( targetheading, 2) + spc AT (18, 10).
    PRINT ": " +  ROUND( cheading, 2) + spc AT (18, 11).

    IF DEFINED route AND route:LENGTH <> 0 AND rwaypoint <> route:LENGTH-1 AND rwaypoint <> -1 {
      PRINT ": " +  round(ABS(route[rwaypoint+1]:HEADING)) + spc AT (18, 12).
      PRINT ": " +  round(nextWaypointHeading) + spc AT (18,13).
    }
    PRINT ": " +  ROUND(__grid:DISTANCE, 2) + spc AT (18, 15).
    PRINT ": " +  route:LENGTH + spc AT (18, 16).
    PRINT ": " +  (rwaypoint+1) + spc AT (18, 17).

    PRINT ": " +  round(currentSlopeAngle,2) + spc AT (18,20).
    PRINT ": " +  round(angle,2) + spc AT (18,21).
    PRINT ": " +  round(pangle,2) + spc AT (18,22).

    PRINT ": " +  round(stopDistance,2) + spc AT (18,24).
    PRINT ": " +  round(gradient,4) + spc AT (18,25).

    PRINT ": " +  Runmode + spc AT (18, 27).
    PRINT ": " +  AG1 + "   " AT (18, 28).

    PRINT ": " +  ROUND( settings["Odometer"]/1000, 1) + " km" + spc AT (18, 30).
    if lastCharge <= Electric:AMOUNT {
      PRINT ": +" AT (18,31).
    } ELSE {
      PRINT ": -" AT (18,31).
    }
    PRINT ROUND( chargeLevel, 1) + "%" + spc AT (22, 31).
    if ABS(GROUNDSPEED) > 0 AND targetspeed <> 0 {
      PRINT ": " +  ROUND( get_slope_speed(), 2) + spc AT (18,34).
    }

    PRINT ": " +  AG2 + "   " AT (18, 35).
    PRINT ": " + ROUND(wtVAL,3) + spc AT (37,10).
    PRINT ": " + ROUND(kTurn,3) + spc AT (37,11).
    PRINT ": " + ROUND(turnlimit,3) + spc AT (37,12).
    PRINT ": " + ROUND(dir,2) + spc AT (37,13).
  }

  SET looptime TO TIME:SECONDS - loopEndTime.
  SET loopEndTime TO TIME:SECONDS.

  SET settings["Odometer"] TO settings["Odometer"] + (GROUNDSPEED * looptime).
  if TIME:SECONDS > nextWrite {
    WRITEJSON(settings,"1:/config/settings.json").
    SET nextWrite TO TIME:SECONDS + 10.
  }
  // WAIT 0. //  We only need to run an iteration once per physics tick.  Ensure that we pause until the next tick.
}

FUNCTION display_HUD {
  CLEARSCREEN.
  PRINT header AT (0,1).
  PRINT "Target Speed" AT (2,6).
  PRINT "Surface Speed" AT (2,7).
  PRINT "Distance to goal" AT (2,8).

  PRINT "Target Heading" AT (2,10).
  PRINT "CurrentHeading" AT (2,11).
  PRINT "Next Heading" AT (4,12).
  PRINT "Next Bearing" AT (4,13).

  PRINT "Waypoint Dist"  AT (2,15).
  PRINT "Waypoints" AT (2,16).
  PRINT "Current WP" AT (2,17).

  PRINT "Current Angle" AT (2,20).
  PRINT "Predicted Angle" AT (2,21).

  PRINT "Difference" AT (2,22).
  PRINT "Stopping Dist" AT (2,24).
  PRINT "Gradient" AT (2,25).

  PRINT "Runmode" AT (2,27).
  PRINT "Cruise Control" AT (2,28).

  PRINT "Odometer" AT (2,30).
  PRINT "Electric" AT (2,31).

  PRINT "Slope Speed" AT (2,34).
  PRINT "Send Data" AT (2,35).
  PRINT "wTVAL" AT (32,10).
  PRINT "kTurn" AT (32,11).
  PRINT "Limit" AT (32,12).
  PRINT "Dir" AT (32,13).

  PRINT "Roverware Version : " + settings["Version"] AT (2,TERMINAL:HEIGHT-1).
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

FUNCTION nav_lines {
  LOCAL c IS 0.
  LOCAL wl IS false.
  if wpm:LENGTH <> 0 {
    SET wl TO true.
  }
  FOR w IN route {
    if c < route:LENGTH-1 {
      if wl = true {
        SET wpm[c]:START TO w:ALTITUDEPOSITION(w:TERRAINHEIGHT+5).
        SET wpm[c]:VEC TO route[c+1]:POSITION - route[c+1]:ALTITUDEPOSITION(route[c+1]:TERRAINHEIGHT+5).
      } else {
        wpm:ADD(VECDRAWARGS(w:ALTITUDEPOSITION(w:TERRAINHEIGHT+5),route[c+1]:POSITION - route[c+1]:ALTITUDEPOSITION(route[c+1]:TERRAINHEIGHT+5),GREEN)).
      }
      SET c TO c + 1.
    }
  }
}

FUNCTION start_navigation
{
  SET TERMINAL:WIDTH TO 50.
  SET TERMINAL:HEIGHT TO 40.
  CLEARSCREEN.
  WRITEJSON(settings,"1:/config/settings.json").
  display_HUD().
  if route:LENGTH <> 0 {
  SET rwaypoint TO 1.
    SET __grid TO LATLNG(route[rwaypoint]:LAT,route[rwaypoint]:LNG).
    LOCK targetHeading TO __grid:HEADING.
    SET runmode TO 0.
    SET AG1 TO TRUE.
    restore_speed().
    CLEARVECDRAWS().
  }
}

FUNCTION set_speed
{
  PARAMETER spd, mode is -1.
  if spd < targetspeed {
    SET targetspeed TO spd.
  }
  if mode <> -1 {
    SET runmode TO mode.
    SET lastEvent[mode] TO TIME:SECONDS.
  }
}

FUNCTION restore_speed
{
  SET targetspeed TO settings["DefaultSpeed"].
  SET lastEvent[0] TO TIME:SECONDS.
}

FUNCTION get_stop_distance {
  PARAMETER speed, gr IS gradient.
  return speed^2 / (2 * const_gravity * coList[BODY:NAME] + gr).
}

FUNCTION get_slope_speed {
  return SQRT(
      MAX(settings["DefaultSpeed"]/2,
        ABS(get_stop_distance(settings["DefaultSpeed"],0))
        *
        (
          2 * const_gravity
          *
          coList[BODY:NAME] + gradient
        )
      )
    ).
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

FUNCTION hold_poition
{
  PARAMETER mode.
  set_speed(0,mode).
  BRAKES ON.
  SET brakesOn TO TRUE.
  SET SHIP:CONTROL:NEUTRALIZE TO TRUE.
}

FUNCTION restore_operations
{
  SET runmode TO 0.
  CLEARSCREEN.
  display_HUD().
  if route:LENGTH <> 0 {
    restore_speed().
    SET brakesOn TO FALSE.
    SET WARP TO 0.
  }
  SET lastEvent[0] TO TIME:SECONDS.
}

FUNCTION load_map
{
  SET menu TO 5.
  LOCAL mp IS READJSON("0:/astar-rover/backup/map.json").
  LOCAL l IS mp:LENGTH.
  IF l > 160 {
    SET TERMINAL:WIDTH TO (l/1.5) + 10.
    SET TERMINAL:HEIGHT TO (l/1.5) + 10.
    SET TERMINAL:CHARHEIGHT TO 6.
  } else {
    SET TERMINAL:WIDTH TO (l) + 10.
    SET TERMINAL:HEIGHT TO (l) + 10.
  }
  LOCAL y IS 0.
  FOR row IN mp {
    PRINT row AT (0,y).
    SET y TO y + 1.
  }
}
