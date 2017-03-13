CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
IF ADDONS:RT:AVAILABLE
{
  CLEARSCREEN.
  PRINT "-- {    WAITING FOR CONNECTION    }---" AT (2,2).
  PRINT "Acquiring signal " AT (2,5).
  LOCAL c IS 19.
  UNTIL ADDONS:RT:HASKSCCONNECTION(SHIP) {
    PRINT "." AT (c,5).
    SET c TO c + 1.
    WAIT 1.
    IF c = 25 {
      PRINT "       " AT (19,5).
      SET c TO 19.
    }
  }
  SET WARP TO 0.
}
LOCAL vol IS "1:".
IF EXISTS("1:/astar-rover") = FALSE {
  SET vol TO "0:".
}
CLEARSCREEN.
runpath(vol+"/astar-rover/rover").
