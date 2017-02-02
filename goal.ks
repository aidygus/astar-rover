
PARAMETER debug IS true.

SET AG9 TO FALSE.

clearscreen.
CLEARVECDRAWS().
SET runmode TO 0.
SET __goal TO SHIP:GEOPOSITION.
nav_marker().
on ag10 { //When the 0 key IS pressed:
    // End the program
    SET runmode TO -1.
    }

until runmode = -1 {

    //Handle User Input using action groups
        IF AG2 { // Increment the Latitude
            SET __goal TO LATLNG(__goal:LAT+0.1,__goal:LNG).
            nav_marker().
            SET AG2 TO FALSE. //Reset the AG after we read it
            }
        ELSE IF AG3 { // Decrease the Latitude
            SET __goal TO LATLNG(__goal:LAT-0.1,__goal:LNG).
            nav_marker().
            SET AG3 TO FALSE.
            }
        ELSE IF AG4 { // Increase the Longitude
            SET __goal TO LATLNG(__goal:LAT,__goal:LNG+0.1).
            nav_marker().
            SET AG4 TO FALSE.
             }
        ELSE IF AG5 { //  Decrease the Longitude
            SET __goal TO LATLNG(__goal:LAT,__goal:LNG-0.1).
            nav_marker().
             SET AG5 TO FALSE.
             }
        ELSE IF AG6 { // Reset marker to ship position
             SET __goal TO SHIP:GEOPOSITION.
             nav_marker().
             SET AG6 TO FALSE.
             }
      ELSE IF AG9 { // Execute the astar script
        CLEARSCREEN.
        RUNPATH("/asrover/astar","LATLNG",__goal,debug).
        CLEARSCREEN.
        SET AG9 TO FALSE.
        CLEARVECDRAWS().
        // RUNPATH("/yourscript"). // Execute your rover management script here
      }
    }

    FUNCTION nav_marker {
      SET vg TO VECDRAWARGS(
                  __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  __goal:POSITION - __goal:ALTITUDEPOSITION(__goal:TERRAINHEIGHT+1000),
                  Green, "", 1, true,50).
    }
