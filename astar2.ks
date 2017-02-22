PARAMETER input1 IS 1, input2 IS 1, debug IS true.

SET len to 100.                    // Size of the graph
SET sindex TO 25.                  // Starting Y position in the graph
SET gindex TO CEILING((len-1)/2). //  Grid reference for the center of the graph which is the goal

SET start TO SHIP:GEOPOSITION.              // Get starting POSITION
if input1 = "LATLNG" {
  SET goal TO LATLNG(input2:LAT,input2:LNG).
  SET gDist TO MAX(45,CEILING((goal:DISTANCE*1.2)/100)).
  SET len TO gDist * 2.
  SET sindex TO FLOOR(len/5).
  SET gindex TO gDist.
} else if input = "WAYPOINT" {
  SET wp TO WAYPOINT(input2).
  SET goal TO wp:GEOPOSITION.
} else {
  SET goal TO LATLNG(start:LAT+input1,start:LNG+input2).  // Specify the physical lat/lan of the goal.
}

CLEARSCREEN.
PRINT "Initializing".
PRINT "Graph Size   : " + len.
PRINT "Starting Ref : " + sindex.
SET TERMINAL:WIDTH TO len + 10.
SET TERMINAL:HEIGHT TO len + 10.


SET map to LEXICON().  // Node list
SET openset TO LEXICON(). // Open Set
SET closedset TO LEXICON(). // Closed Set
SET fscorelist TO LEXICON().// fscore list
SET fscore TO LEXICON().
SET gscore TO LEXICON().
SET camefrom TO LEXICON().
SET neighbourlist TO LIST(LIST(1,0),LIST(1,-1),LIST(0,-1),LIST(-1,1),LIST(-1,0),LIST(-1,-1),LIST(0,1),LIST(1,1)).  // Neighbours of cells.

// Create a list describing the grid

// FROM {local x is 0.} UNTIL x = len STEP {set x to x+1.} DO {
//   n:ADD(LIST(999999999,0,0,0,0,LIST())).
// }
// FROM {local x is 0.} UNTIL x = len STEP {set x to x+1.} DO {
//   l:ADD(n:COPY).
// }



SET latdist TO (goal:LAT-start:LAT) / (gindex-sindex).  //  Calculate the distance between each graph segment
SET lngdist TO (goal:LNG-start:LNG) / (gindex-sindex).

CLEARSCREEN.
CLEARVECDRAWS().
SET vex TO LIST().

place_marker(start,red,5).
place_marker(goal,green,100,1000).

SET route TO astar(LIST(sindex,gindex),LIST(gindex,gindex)).
if route:LENGTH <> 0 {
  CLEARVECDRAWS().
  SET route TO navigation_points(route).
} else {
  PRINT "Route can not be found".
}
SET TERMINAL:WIDTH TO 30.
SET TERMINAL:HEIGHT TO 40.


//    /**
//    @current LIST coordinates of the starting cell in the graph
//
//    @return LIST/Boolean  Either returns a constructed list of the discovered route or false if no valid route can be found.
//    **/

FUNCTION astar {
  PARAMETER start, goal.
  // Estimate the Heuristic cost of getting to the goal.
  SET estimated_heuristic TO heuristic_cost_est(LIST(sindex,gindex),gindex).


  // Add starting position to open list
  openset:ADD(sindex+","+gindex,TRUE).

  SET map[sindex+","+gindex] TO LEXICON("LAT",start:LAT,"LNG",start:LNG,"TERRAINHEIGHT",start:TERRAINHEIGHT,"POSITION",start:POSITION).
  SET map[gindex+","+gindex] TO LEXICON("LAT",goal:LAT,"LNG",goal:LNG,"TERRAINHEIGHT",goal:TERRAINHEIGHT,"POSITION",goal:POSITION).

  SET gscore[sindex+","+gindex] TO 0.
  SET fscorelist[estimated_heuristic] TO LEXICON(sindex+","+gindex,LIST(sindex,gindex)).
  SET fscore[sindex+","+gindex] TO estimated_heuristic.

  PRINT "G" AT (gindex,gindex).

  until openset:LENGTH = 0 {

    LOCAL fscores IS fscorelist:KEYS.
    LOCAL localfscore IS len*len.
    LOCAL delscore IS "".
    FOR score in fscores {
      if score > localfscore {
        LOCAL scorekey IS fscorelist[score]:KEYS.
        LOCAL current IS fscorelist[score][scorekey[0]].
        SET delscore TO scorekey[0]
        SET localfscore TO score.
      }
    }

    fscorelist[localfscore]:REMOVE(delscore).

    PRINT "Grid       : " + current[0]+":"+current[1] AT (5,64).
    PRINT "Open Set   : " + os:LENGTH AT (5,65).
    PRINT "Closed Set : " + cs:LENGTH AT (5,66).

    if current[0] = gindex and current[1] = gindex {
      return construct_route().
      BREAK.
    }
    openset:REMOVE(current[0]+","+current[1]).
    closedset:ADD(current[0]+","+current[1],TRUE).
    get_neighbours(current,gscore[current[0]+","+current[1]]).
  }
  return LIST().
}

FUNCTION get_neighbours {
  PARAMETER current,currentgscore.

  LOCAL node IS map[current[0]+","+current[1]].
  LOCAL scount IS 0.    //  Counter for neighbours with bad slopes

      //  Work through the neighbour list starting directly ahead and working around clockwise.
  FOR ne IN nf {
    // if scount = 0 {
    LOCAL gridy IS current[0] + ne[0].
    LOCAL gridx IS current[1] + ne[1].
    LOCAL neighbour IS gridy+","+gridx.

    if closedset:HASKEY(neighbour) {
      // Continue and do nothing
    } else {
      if test_neighbour(current,ne) {
        LOCAL tentative_gscore IS gscore[current[0]+","+current[1]] + 1.
          //  We don't want to fall off the grid!
        if gridy >= 0 AND gridy <= len-1 AND gridx >= 0 AND gridx <= len-1 {
          if openset:HASKEY(neighbour) = FALSE {
            openset:ADD(neighbour,TRUE).
          } else if tentative_gscore >= currentfscore {
            //  This is not a better path.  Do nothing.
          }
          SET camefrom[neighbour] TO current.
          SET gscore[neighbour] TO tentative_gscore.
          SET fscore[neighbour] TO gscore[neighbour] + heuristic_cost_est(LIST(gridy,gridx),gindex).

          if fscorelist:HASKEY(fscore[neighbour]) {
            if fscorelist[fscore[neighbour]]:HASKEY(neighbour) = FALSE {
              fscorelist[fscore[neighbour]]:ADD(neighbour,LIST(gridy,gridx)).
            }
          } else {
            fscorelist:ADD(fscore[neighbour],LEXICON(neighbour,LIST(gridy,gridx))).
          }
        }
      } else {
        SET scount TO scount + 1.
      }
    }
    //  If there are 3 slopes neighbouring then mark it as bad and go back to previous cell
    if scount = 3 {
      openset:REMOVE(current[0]+","+current[1]).
      closedset:ADD(current[0]+","+current[1],TRUE).
      PRINT "!" AT (current[1],current[0]).
      SET current TO camefrom[current[0]+","+current[1]].
      BREAK.
    }
  }
}

FUNCTION test_neighbour{
  PARAMATER current, ne.

  // This bit is nasty!  It took me more than a few hours to balance it right and the ne[0] bit is a hack and a half but it works.
  LOCAL offsetx IS 0.
  LOCAL offsety IS 0.
  LOCAL node IS map[current[0]+","+current[1]]
  LOCAL _grid IS LATLNG(node["LAT"],node["LNG"]).
  if ne[0] <> 0 {
    if ne[1] = 0 {
      SET offsety TO (ne[0]*latdist)/2.
      SET offsetx TO (ne[0]*lngdist)/2.
    } else {
      SET offsety TO (ne[0]*latdist).
      SET offsetx TO (ne[0]*lngdist).
    }
  }
  SET _grid TO LATLNG(node["LAT"]+offsety,node["LNG"]+offsetx).
  if ne[1] <> 0 {
    SET offsety TO (ne[1]*lngdist).
    SET offsetx TO -(ne[1]*latdist).
  }

  LOCAL grid IS LATLNG(_grid:LAT+offsety,_grid:LNG+offsetx).
  LOCAL heightdiff IS grid:TERRAINHEIGHT-node["TERRAINHEIGHT"].

  //  We want to avoid trying to drive up or down cliffs and especially taking a dip if we can help it
  LOCAL setlist TO 0.
  LOCAL distance IS (grid:POSITION-node["POSITION"]):MAG.
  LOCAL angle IS ARCSIN(heightdiff/distance).
  if angle > -5 AND angle < 15 AND grid:TERRAINHEIGHT >= 0 {
      PRINT "." AT (gridx,gridy).
      place_marker(grid,yellow,5,100,round(angle),0.05).
      // os:ADD(LIST(chk,gridy, gridx,_fscore)).
      SET setlist TO 1.
  } else if grid:TERRAINHEIGHT < 0 {
    PRINT "!" AT (gridx,gridy).
    place_marker(grid,red,5,100,round(angle),0.05).
    // cs:ADD(LIST(chk,gridy, gridx,_fscore)).
    SET setlist TO 2.
  } else {
    if angle <= -5 {
      PRINT "v" AT (gridx, gridy).
    } else {
      PRINT "^" AT (gridx, gridy).  // Do Nothing for now, highlight cell has been touched visially but is not a valid route from this point
    }
  }
  // Update the graph with what we've discovered about this cell.

  SET map[gridy+","+gridx] TO LEXICON(
    "LAT",grid:LAT,
    "LNG",grid:LNG,
    "TERRAINHEIGHT",grid:TERRAINHEIGHT,
    "POSITION",grid:POSITION
  ).
  if setlist = 1 {
    return TRUE.
  } else {
    return FALSE.
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
  LOCAL current is LIST(gindex,gindex).
  LOCAL totalpath IS LIST(current).
  WHILE camefrom:HASKEY(current) {
    SET current TO camefrom[current[0]+","+current[1]].
    PRINT "*" AT (current[1],current[0]).
    place_marker(LATLNG(map[current[0]+","+current[1]]["LAT"],map[current[0]+","+current[1]]["LNG"]),yellow,1,100,"",30).
    totalpath:ADD(LIST(current[0],current[1])).
  }
  return totalpath.
}

FUNCTION navigation_points {
  PARAMETER path.
  LOCAL points IS LIST().
  FOR p in path {
    if p:LENGTH = 2 {
      points:INSERT(0,LIST(LATLNG(map[p[0]+","+p[1]]["LAT"],map[p[0]+","+p[1]]["LNG"]))).
    }
  }
  return points.
}
