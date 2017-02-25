IF EXISTS("1:/asrover") = FALSE {
  SWITCH TO 0.
}
CORE:PART:GETMODULE("kOSProcessor"):DOEVENT("Open Terminal").
CLEARSCREEN.

PRINT "Initializing".
runpath("/asrover/rover").
