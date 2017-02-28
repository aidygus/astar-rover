
LOCAL s IS get_science_parts().
SET scrunmode TO 0.

until scrunmode = -1 {
    if scrunmode = 0 {
      science_menu().
    } else if scrunmode = 1 {
      do_science().
  }
}




FUNCTION get_science_parts {
  return SHIP:PARTSTAGGED("Skience").

}

FUNCTION science_menu {
  CLEARSCREEN.
  PRINT "  ---{  Select a science part }---" AT (2,2).
  LOCAL y IS 1.
  FOR p IN s {
    PRINT "(" + y + ") " + p:TITLE AT (4,y+4).
    PRINT "" AT (4,y+5).
    SET y TO y + 2.
  }
  SET scrunmode TO 1.
}

FUNCTION do_science {
  IF TERMINAL:INPUT:HASCHAR {
    LOCAL K IS TERMINAL:INPUT:GETCHAR().
    LOCAL N IS K:TONUMBER(-99).
    IF N <> -99 {
      LOCAL sc TO s[N-1].
      LOCAL mods IS sc:MODULES.
      FOR mo IN mods {
        if mo = "ModuleScienceExperiment"{
          transmit_science(sc:GETMODULE("ModuleScienceExperiment")).
        } else if mo = "DMModuleScienceAnimate" {
          transmit_science(sc:GETMODULE("DMModuleScienceAnimate")).
        }
      }
    }
  }
}

transmit_science {
  PARAMETER M.

  PRINT "Preparing science part" AT (2,10).
  M:DUMP.
  M:RESET.
  PRINT "Performing Science" AT (2,11).
  WAIT 0.
  M:DEPLOY.
  WAIT 0.
  PRINT "Transmitting Science" AT (2,12).
  if  M:HASDATA {
    M:TRANSMIT.
  }
}
