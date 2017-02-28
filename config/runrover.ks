IF EXISTS("1:/asrover") = FALSE {
  SWITCH TO 0.
} else {
  SWITCH TO 1.
}
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
CLEARSCREEN.

PRINT "Initializing".
runpath("/astar-rover/rover").
