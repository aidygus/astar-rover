IF ADDONS:RT:AVAILABLE
{
  CLEARSCREEN.
  PRINT "-- {    WAITING FOR CONNECTION    }---" AT (2,2).
  WAIT UNTIL ADDONS:RT:HASKSCCONNECTION(SHIP).
}
IF EXISTS("1:/astar-rover") = FALSE {
  SWITCH TO 0.
} else {
  SWITCH TO 1.
}
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
CLEARSCREEN.

PRINT "Initializing".
runpath("/astar-rover/rover").
