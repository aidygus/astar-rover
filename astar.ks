PARAMETER input1 IS 1, input2 IS 1, debug IS true.

LOCAL asrunmode IS 0.
LOCAL current_ipu IS CONFIG:IPU.

SET CONFIG:IPU TO settings["IPU"].

LOCAL goal IS "".
LOCAL wp IS "".

LOCAL charSize IS LIST(TERMINAL:CHARHEIGHT,TERMINAL:CHARWIDTH).

LOCAL start IS SHIP:GEOPOSITION.              // Get starting POSITION

if input1 = "LATLNG" {
  SET goal TO LATLNG(input2:LAT,input2:LNG).
} else if input1 = "WAYPOINT" {
  SET wp TO WAYPOINT(input2).
  SET goal TO wp:GEOPOSITION.
} else {
  SET goal TO LATLNG(start:LAT+input1,start:LNG+input2).  // Specify the physical lat/lan of the goal.
}

LOCAL len IS MAX(50,MIN(300,CEILING((goal:DISTANCE/100)*3))).


  SET TERMINAL:CHARWIDTH TO 4.
  SET TERMINAL:CHARHEIGHT TO 4.
}
LOCAL gDist IS CEILING(goal:DISTANCE/(len/3)).
LOCAL gindex IS CEILING((len-1)/2).  //  Grid reference for the center of the graph which is the goal
LOCAL sindex IS gindex - FLOOR(goal:DISTANCE/gDist).

CLEARSCREEN.
PRINT "Initializing".
PRINT "Graph Size   : " + len.
PRINT "Starting Ref : " + sindex.

LOCAL len IS MAX(50,MIN(300,CEILING((goal:DISTANCE/100)*3))).
IF len > 160 {
  SET TERMINAL:WIDTH TO (len/2) + 10.
  SET TERMINAL:HEIGHT TO (len/2) + 10.
  SET TERMINAL:CHARWIDTH TO 4.
  SET TERMINAL:CHARHEIGHT TO 4.
} else {
  SET TERMINAL:WIDTH TO len + 10.
  SET TERMINAL:HEIGHT TO len + 10.
}


LOCAL map IS LEXICON().  // Node list
LOCAL openset IS LEXICON(). // Open Set
LOCAL closedset IS LEXICON(). // Closed Set
LOCAL fscorelist IS LEXICON().// fscore list
LOCAL fscore IS LEXICON().
LOCAL gscore IS LEXICON().
LOCAL camefrom IS LEXICON().
LOCAL neighbourlist IS LIST(LIST(1,0),LIST(1,-1),LIST(0,-1),LIST(-1,1),LIST(-1,0),LIST(-1,-1),LIST(0,1),LIST(1,1)).  // Neighbours of cells.

LOCAL latdist IS (goal:LAT-start:LAT) / (gindex-sindex).  //  Calculate the distance between each graph segment
LOCAL lngdist IS (goal:LNG-start:LNG) / (gindex-sindex).

CLEARSCREEN.
CLEARVECDRAWS().
LOCAL vex IS LIST().

place_marker(start,red,5).
place_marker(goal,green,100,1000).

GLOBAL route IS astar(sindex,gindex).
CLEARVECDRAWS().
clear_down().
if route:LENGTH = 0 {
  PRINT "---{  Route can not be found  }---" AT (2,1).
}
SET TERMINAL:WIDTH TO 50.
SET TERMINAL:HEIGHT TO 40.
SET CONFIG:IPU TO current_ipu.
SET TERMINAL:CHARWIDTH TO charSize[1].
SET TERMINAL:CHARHEIGHT TO charSize[0].

//    /**
//    @sindex coordinates of the starting x cell in the graph
//    @gindex coordinates of the y call which is in the middle of the graph and location of the goal on both x and y.
//
//    @return LIST  returns a constructed list of the discovered route or empty if none is found.
//    **/

FUNCTION astar {
  PARAMETER sindex, gindex.
  // Estimate the Heuristic cost of getting to the goal.
  LOCAL estimated_heuristic IS heuristic_cost_est(LIST(sindex,gindex),gindex).
  LOCAL current IS LIST(sindex,gindex).

  // Add starting position to open list
  openset:ADD(sindex+","+gindex,TRUE).

  SET map[sindex+","+gindex] TO LEXICON("LAT",start:LAT,"LNG",start:LNG,"TERRAINHEIGHT",start:TERRAINHEIGHT,"POSITION",start:POSITION,"FSCORE",estimated_heuristic).
  SET map[gindex+","+gindex] TO LEXICON("LAT",goal:LAT,"LNG",goal:LNG,"TERRAINHEIGHT",goal:TERRAINHEIGHT,"POSITION",goal:POSITION,"FSCORE",0).

  SET gscore[sindex+","+gindex] TO 0.
  SET fscorelist[estimated_heuristic] TO LEXICON(sindex+","+gindex,LIST(sindex,gindex)).
  SET fscore[sindex+","+gindex] TO estimated_heuristic.


  until openset:LENGTH = 0 OR asrunmode = -1 {

    LOCAL fscores IS fscorelist:KEYS.
    LOCAL localfscore IS len*len.
    LOCAL delscore IS "".

    FOR score in fscores {
      if fscorelist[score]:LENGTH = 0 {
        fscorelist:REMOVE(score).
      } else if score < localfscore {
        LOCAL scorekey TO fscorelist[score]:KEYS.
        if closedset:HASKEY(scorekey[0]) = FALSE {
          SET current TO fscorelist[score][scorekey[0]].
          SET delscore TO scorekey[0].
          SET localfscore TO score.
        } else {
          fscorelist[score]:REMOVE(scorekey[0]).
        }
      }
    }
    if fscorelist:HASKEY(localfscore) {
      fscorelist[localfscore]:REMOVE(delscore).
    }
    if closedset:HASKEY(current[0]+","+current[1]) = FALSE {
      PRINT "S" AT (gindex,sindex).
      PRINT "G" AT (gindex,gindex).
      PRINT "IPU        : " + CONFIG:IPU AT (5,63).
      PRINT "Grid       : " + current[0]+":"+current[1] AT (5,64).
      PRINT "Open Set   : " + openset:LENGTH AT (5,65).
      PRINT "Closed Set : " + closedset:LENGTH AT (5,66).
      PRINT "fScore     : " + localfscore + "   " AT (5,67).
      PRINT "Map size   : " + map:LENGTH + "   " AT (5,68).
      PRINT "                                              " AT (5,70).

      // WRITEJSON(camefrom,"0:/camefrom.json").
      if current[0] = gindex and current[1] = gindex {
        return construct_route().
        BREAK.
      }
      openset:REMOVE(current[0]+","+current[1]).
      closedset:ADD(current[0]+","+current[1],TRUE).
      get_neighbours(current,gscore[current[0]+","+current[1]]).
    } else {
      PRINT current[0]+","+current[1]+" is in closed list" AT (5,70).
      openset:REMOVE(current[0]+","+current[1]).
    }

    IF TERMINAL:INPUT:HASCHAR {
      LOCAL K IS TERMINAL:INPUT:GETCHAR().
      IF K = TERMINAL:INPUT:ENDCURSOR {
        SET asrunmode TO -1.
      }
    }
  }
  return LIST().
}



FUNCTION get_neighbours {
  PARAMETER current,currentgscore.

  LOCAL node IS map[current[0]+","+current[1]].
  LOCAL scount IS 0.    //  Counter for neighbours with bad slopes

      //  Work through the neighbour list starting directly ahead and working around clockwise.
  FOR ne IN neighbourlist {
    // if scount = 0 {
    LOCAL gridy IS current[0] + ne[0].
    LOCAL gridx IS current[1] + ne[1].
    LOCAL neighbour IS gridy+","+gridx.

    if closedset:HASKEY(neighbour) {
      // Continue and do nothing
    } else {
      if test_neighbour(current,ne,LIST(gridx,gridy)) {
        LOCAL tentative_gscore IS gscore[current[0]+","+current[1]] + 1 + map[neighbour]["WEIGHT"].
          //  We don't want to fall off the grid!
        if gridy >= 0 AND gridy <= len-1 AND gridx >= 0 AND gridx <= len-1 {
          if openset:HASKEY(neighbour) = FALSE {
            openset:ADD(neighbour,TRUE).
          } else if tentative_gscore >= currentgscore {
            //  This is not a better path.  Do nothing.
          }

          SET camefrom[neighbour] TO current.
          SET gscore[neighbour] TO tentative_gscore.
          SET fscore[neighbour] TO gscore[neighbour] + heuristic_cost_est(LIST(gridy,gridx),gindex).
          SET map[neighbour]["FSCORE"] TO fscore[neighbour].

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
      // If there are 3 slopes neighbouring then mark it as bad and go back to previous cell
      // if scount = 3 {
      //   PRINT "!" AT (current[1],current[0]).
      //   BREAK.
      // }
    }
  }
}

FUNCTION test_neighbour{
  PARAMETER current, ne, printat.

  // This bit is nasty!  It took me more than a few hours to balance it right and the ne[0] bit is a hack and a half but it works.
  LOCAL offsetx IS 0.
  LOCAL offsety IS 0.
  LOCAL node IS map[current[0]+","+current[1]].
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
  LOCAL weight TO 0.
  if angle > settings["MinSlope"] AND angle < settings["MaxSlope"] AND ROUND(grid:TERRAINHEIGHT) >= 0 {
      PRINT "." AT (printat[0],printat[1]).
      place_marker(grid,yellow,5,100,round(angle),0.05).
      SET setlist TO 1.
  } else if grid:TERRAINHEIGHT < 0 {
    PRINT "!" AT (printat[0],printat[1]).
    place_marker(grid,red,5,100,round(angle),0.05).
    SET setlist TO 2.
  } else {
    if angle <= settings["MinSlope"] {
      PRINT "v" AT (printat[0],printat[1]).
      SET weight TO 1.
    } else {
      PRINT "^" AT (printat[0],printat[1]).  // Do Nothing for now, highlight cell has been touched visially but is not a valid route from this point
      SET weight TO 1.
    }
  }
  // Update the graph with what we've discovered about this cell.

  SET map[printat[1]+","+printat[0]] TO LEXICON(
    "LAT",grid:LAT,
    "LNG",grid:LNG,
    "TERRAINHEIGHT",grid:TERRAINHEIGHT,
    "POSITION",grid:POSITION,
    "FSCORE",0,
    "WEIGHT",weight
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
  LOCAL totalpath IS LIST(
          LATLNG(
            map[current[0]+","+current[1]]["LAT"],
            map[current[0]+","+current[1]]["LNG"]
          )
        ).
  UNTIL camefrom:HASKEY(current[0]+","+current[1]) = FALSE {
    SET current TO camefrom[current[0]+","+current[1]].
    PRINT "*" AT (current[1],current[0]).
    place_marker(LATLNG(map[current[0]+","+current[1]]["LAT"],map[current[0]+","+current[1]]["LNG"]),yellow,1,100,"",30).
    totalpath:INSERT(0,LATLNG(map[current[0]+","+current[1]]["LAT"],map[current[0]+","+current[1]]["LNG"])).
  }
  CLEARSCREEN.
  return totalpath.
}

FUNCTION clear_down {
  CLEARVECDRAWS().
  map:CLEAR.
  openset:CLEAR.
  closedset:CLEAR.
  fscorelist:CLEAR.
  fscore:CLEAR.
  gscore:CLEAR.
  camefrom:CLEAR.
}

FUNCTION center {
  PARAMETER string,y.

  LOCAL x IS ROUND(TERMINAL:WIDTH / 2) - FLOOR(string:LENGTH / 2).
  PRINT string AT (x,y).
}
