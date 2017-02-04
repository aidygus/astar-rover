PARAMETER input1 IS 1, input2 IS 1, debug IS true.

CLEARSCREEN.
SET len to 10.                    // Size of the graph
SET sindex TO 2.                  // Starting Y position in the graph
SET gindex TO CEILING((len-1)/2). //  Grid reference for the center of the graph which is the goal
SET TERMINAL:WIDTH TO len + 10.
SET TERMINAL:HEIGHT TO len + 10.


SET n to LIST().  // Template list for rows
SET l to LIST().  // Node list
SET os TO LIST(). // Open Set
SET cs TO LIST(). // Closed Set
SET nf TO LIST(LIST(1,0),LIST(1,-1),LIST(0,-1),LIST(-1,1),LIST(-1,0),LIST(-1,-1),LIST(0,1),LIST(1,1)).  // Neighbours of cells.

// Create a list describing the grid

FROM {local x is 0.} UNTIL x = len STEP {set x to x+1.} DO {
  n:ADD(LIST(999999999,0,0,0,0,LIST())).
}
FROM {local x is 0.} UNTIL x = len STEP {set x to x+1.} DO {
  l:ADD(n:COPY).
}

SET start TO SHIP:GEOPOSITION.              // Get starting POSITION
if input1 = "LATLNG" {
  SET goal TO LATLNG(input2:LAT,input2:LNG).
} else if input = "WAYPOINT" {
  SET wp TO WAYPOINT(input2).
  SET goal TO wp:GEOPOSITION.
} else {
  SET goal TO LATLNG(start:LAT+input1,start:LNG+input2).  // Specify the physical lat/lan of the goal.
}

SET latdist TO (goal:LAT-start:LAT) / (gindex-sindex).  //  Calculate the distance between each graph segment
SET lngdist TO (goal:LNG-start:LNG) / (gindex-sindex).

CLEARVECDRAWS().
SET vex TO LIST().

place_marker(start,red,5).
place_marker(goal,green,100,1000).

// Estimate the Heuristic cost of getting to the goal.
SET esth TO heuristic_cost_est(LIST(sindex,gindex),gindex).

// gscore, fscore, lat/lan, height, list and origin
// List memberships is 0 (none), 1 (open set), 2(closed set)
SET l[sindex][gindex] TO LIST(0,esth,start,start:TERRAINHEIGHT,0,LIST()).
SET l[gindex][gindex] TO LIST(esth,0,goal,goal:TERRAINHEIGHT,0,LIST()).

// Add starting position to open list
os:ADD(LIST(sindex+","+gindex,sindex,gindex,esth)).
PRINT "G" AT (gindex,gindex).

// Attempt a direct route to goal first.  This is the quickest method which stops if it hits an obstruction.
// SET current_grid TO direct_route(LIST(sindex,gindex)).
//
// if current_grid[0] = gindex {
//   PRINT "Direct route found".
// } else {
  // SET os TO LIST().
  // os:ADD(LIST(sindex+","+gindex,sindex,gindex,esth)).
  // SET cs TO LIST().
  SET route TO get_neighbours(LIST(sindex,gindex)).
  if route:LENGTH <> 0 {
    CLEARVECDRAWS().
    SET route TO navigation_points(route).
  } else {
    PRINT "Route can not be found".
  }
  SET TERMINAL:WIDTH TO 30.
  SET TERMINAL:HEIGHT TO 40.
  // return navigation_path.
// }
//


//  /**
//    @camefrom LIST() the origin graph coordinates of the starting cell
//  **/
FUNCTION direct_route {
  PARAMETER camefrom.
  LOCAL y IS 1.
  LOCAL x IS 0.
  UNTIL os:LENGTH = 0 {
    LOCAL current IS l[camefrom[0]][camefrom[1]].

    cs:ADD(os[0]).  // Add current node to closed set
    os:REMOVE(0).   // Remove current node from open set
    SET l[camefrom[0]][camefrom[1]][4] TO 2.
    LOCAL grid IS LATLNG(start:LAT+(latdist*y),start:LNG+(lngdist*y)).

    LOCAL heightdiff IS current[3]-grid:TERRAINHEIGHT.

    LOCAL gscore IS l[sindex+(y-1)][gindex][0]+1.
    LOCAL fscore IS gscore + heuristic_cost_est(LIST(sindex+y,gindex),gindex).

    SET l[sindex+y][gindex] TO LIST(gscore, fscore, grid, grid:TERRAINHEIGHT, 0 ,camefrom).

    if heightdiff > -50 AND heightdiff < 100 AND grid:TERRAINHEIGHT >= 0 {
      if y+sindex = gindex {
        BREAK.
      }
        PRINT "." AT (gindex,sindex+y).
      vex:ADD(VECDRAWARGS(
                    grid:ALTITUDEPOSITION(grid:TERRAINHEIGHT+100),
                    grid:POSITION - grid:ALTITUDEPOSITION(grid:TERRAINHEIGHT+100),
                    blue, "", 1, true,5)).

      os:ADD(LIST((sindex+y)+","+gindex,sindex+y,gindex,fscore)).
      SET l[sindex+y][gindex][4] TO 1.
      SET camefrom TO LIST(sindex+y,gindex).
      SET y TO y+1.
      // SET x TO x+1.
    } else {
      place_marker(grid,red).
      PRINT "x" AT (gindex,y).
    }
  }
  return LIST(y+sindex,gindex).
}

//    /**
//    @current LIST coordinates of the starting cell in the graph
//
//    @return LIST/Boolean  Either returns a constructed list of the discovered route or false if no valid route can be found.
//    **/

FUNCTION get_neighbours {
  PARAMETER current.
  until os:LENGTH = 0 {
    PRINT "Grid       : " + current[0]+":"+current[1] AT (5,64).
    PRINT "Open Set   : " + os:LENGTH AT (5,65).
    PRINT "Closed Set : " + cs:LENGTH AT (5,66).
    LOCAL fs IS 9999999.
    LOCAL index IS -1.

    // Try to find an entry in the open list with the lowest fscore.
    FROM {local x is 0.} UNTIL x = os:LENGTH STEP {set x to x+1.} DO {
      if os[x][3] < fs {
        SET current TO LIST(os[x][1],os[x][2]).
        SET fs TO l[os[x][1]][os[x][2]][1].
        SET index TO x.
      }
    }
    print "fScore     : " + fs at (5,67).
    SET l[current[0]][current[1]][4] TO 2.
    cs:ADD(os[index]).
    os:REMOVE(index).

    // Are we there yet?
    if current[0] = gindex and current[1] = gindex {
      return construct_route().
      BREAK.
    }
      LOCAL node IS l[current[0]][current[1]].

      //  Work through the neighbour list starting directly ahead and working around clockwise.
      FOR ne IN nf {
        LOCAL gridy IS current[0] + ne[0].
        LOCAL gridx IS current[1] + ne[1].
        LOCAL gsc IS 0.
        LOCAL _fscore IS 99999999.
        LOCAL chk IS "".
        // LOCAL validated IS false.
        //  We don't want to fall off the grid!
        if gridy >= 0 AND gridy <= len-1 AND gridx >= 0 AND gridx <= len-1 {
          SET chk TO gridy + "," + gridx.
          if l[gridy][gridx][4] = 0 {

            SET gsc TO node[0]+1.
            SET _fscore TO heuristic_cost_est(LIST(gridy,gridx),gindex).

            // This bit is nasty!  It took me more than a few hours to balance it right and the ne[0] bit is a hack and a half but it works.

            LOCAL offsetx IS 0.
            LOCAL offsety IS 0.
            LOCAL _grid IS LATLNG(node[2]:LAT,node[2]:LNG).
            if ne[0] <> 0 {
              if ne[1] = 0 {
                SET offsety TO (ne[0]*latdist)/2.
                SET offsetx TO (ne[0]*lngdist)/2.
              } else {
                SET offsety TO (ne[0]*latdist).
                SET offsetx TO (ne[0]*lngdist).
              }
              SET _grid TO LATLNG(node[2]:LAT+offsety,node[2]:LNG+offsetx).
            }
            if ne[1] <> 0 {
              SET offsety TO (ne[1]*lngdist).
              SET offsetx TO -(ne[1]*latdist).
            }

            LOCAL grid IS LATLNG(_grid:LAT+offsety,_grid:LNG+offsetx).
            PRINT grid:bearing AT (5,68).
            LOCAL heightdiff IS grid:TERRAINHEIGHT-node[3].

            //  We want to avoid trying to drive up or down cliffs and especially taking a dip if we can help it
            LOCAL setlist TO 0.
            LOCAL distance IS (grid:POSITION-node[2]:POSITION):MAG.
            LOCAL angle IS ARCSIN(heightdiff/distance).
            if angle > -4 AND angle < 13 AND grid:TERRAINHEIGHT >=0 {
                PRINT "." AT (gridx,gridy).
                place_marker(grid,yellow,5,100,round(angle),0.05).
                os:ADD(LIST(chk,gridy, gridx,_fscore)).
                SET setlist TO 1.
            } else if grid:TERRAINHEIGHT < 0 {
              PRINT "!" AT (gridx,gridy).
              place_marker(grid,red,5,100,round(angle),0.05).
              cs:ADD(LIST(chk,gridy, gridx,_fscore)).
              SET setlist TO 2.
            } else {
              if angle <= -3 {
                PRINT "v" AT (gridx, gridy).
              } else {
                PRINT "^" AT (gridx, gridy).  // Do Nothing for now, highlight cell has been touched visially but is not a valid route from this point
              }
            }
            // Update the graph with what we've discovered about this cell.
            if setlist <> 0 {
              SET l[gridy][gridx] TO LIST(
                gsc,
                _fscore,
                grid,
                grid:TERRAINHEIGHT,
                setlist,
                current
              ).
            }
        }
      }
    }
    if os:LENGTH = 0 {
      return LIST().
    }
  }
}

// /**
//    @st LIST  These contain the x,y coordinates of the cell in the graph
//    @fn Int   Or gindex of the goal cell which is directly center of the graph

//    This is a monotone heuristic I guess and calculates the distance to travel to the goal horizontally and vertically, not diagonally
//    but fear not because the algorythm will drive you diagonally if you so wish
//
//    @return INT   Heuristic cost of getting from graph point to the goal
//  **/

FUNCTION heuristic_cost_est {
  PARAMETER st, fn.
  LOCAL gridy IS 0.
  LOCAL gridx IS 0.
  if st[0] > fn {
    SET gridy TO st[0]-fn.
  } else {
    SET gridy TO fn-st[0].
  }
  if st[1] > fn {
    SET gridx TO st[1]-fn.
  } else {
    SET gridx TO fn-st[1].
  }
    return gridy + gridx.
}

//  /**
//    @grid LATLNG    the 3d world space SPOT
//    @colour COLOR   I'm english, deal with it!
//    @size INT       How big do we want this arrow
//    @height INT     How high do we want the arrow
//    @text VARCHAR   Any text to put down
//    @textsize FLOAT How big should the text be
//  **/

FUNCTION place_marker {
  PARAMETER grid,colour IS blue, size IS 5, height is 100, text is "",textsize is 1.

  if debug = true {
    vex:ADD(VECDRAWARGS(
                grid:ALTITUDEPOSITION(grid:TERRAINHEIGHT+height),
                grid:POSITION - grid:ALTITUDEPOSITION(grid:TERRAINHEIGHT+height),
                colour, text, textsize, true,size)).
  }
}

//  /**
//      Reverse engineer the route from goal to origin
//
//      @return path  LIST()  Graph coordinates of the discovered route
//  **/

function construct_route {
  LOCAL path IS LIST(List(gindex,gindex)).
  LOCAL current is l[gindex][gindex][5].
  path:ADD(current).
  UNTIL current:LENGTH = 0 {
    if current:LENGTH <> 0 {
      PRINT "*" AT (current[1],current[0]).
      place_marker(l[current[0]][current[1]][2],yellow,1,100,"",30).
      SET current TO l[current[0]][current[1]][5].
      path:ADD(current).
    }
  }
  return path.
}

FUNCTION navigation_points {
  PARAMETER path.
  LOCAL points IS LIST().
  FOR p in path {
    if p:LENGTH = 2 {
      points:INSERT(0,LIST(l[p[0]][p[1]][2])).
    }
  }
  return points.
}
