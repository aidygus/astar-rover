PARAMETER debug IS true.

SET AG9 TO FALSE.

clearscreen.
CLEARVECDRAWS().
SET runmode TO 0.
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

  IF runmode = 0 { //Govern the rover

  //Handle User Input using action groups
    IF AG2 { // SET heading TO the __current heading
      SET __goal TO LATLNG(__goal:LAT+0.1,__goal:LNG).
      nav_marker().
      SET AG2 TO FALSE. //ReSET the AG after we read it
    } ELSE IF AG3 { // Decrease the heading
      SET __goal TO LATLNG(__goal:LAT-0.1,__goal:LNG).
      nav_marker().
      SET AG3 TO FALSE.
    }  ELSE IF AG4 { // Increase the heading
      SET __goal TO LATLNG(__goal:LAT,__goal:LNG+0.1).
      nav_marker().
      SET AG4 TO FALSE.
    } ELSE IF AG5 {
      SET __goal TO LATLNG(__goal:LAT,__goal:LNG-0.1).
      nav_marker().
       SET AG5 TO FALSE.
    } ELSE IF AG6 {
       SET __goal TO SHIP:GEOPOSITION.
       nav_marker().
       SET targetspeed TO 0.
       SET route TO LIST().
       SET rwaypoint TO -1.
       SET AG6 TO FALSE.
       //Prevent decrease IF we are increasing
     } ELSE IF AG9 {
      CLEARSCREEN.
      RUNPATH("/asrover/astar","LATLNG",__goal,debug).
      CLEARSCREEN.
      SET rwaypoint TO 0.
      SET AG9 TO FALSE.
      // CLEARVECDRAWS().
      SET runmode TO -1.
    }
    PRINT "Starting Location " + __current:LAT + " : " + __current:LNG AT (5,5).
    PRINT "Target Location " + __goal:LAT + " : " + __goal:LNG AT (5,6).

  }

  FUNCTION nav_marker {
    SET vg TO VECDRAWARGS(
                __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                Green, "", 1, true,50).
  }
