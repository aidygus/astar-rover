LOCAL vol IS "0:".
IF EXISTS("1:/astar-rover/config/soundpack.json") {
  SET vol TO "1:".
}
FUNCTION center {
  PARAMETER s,y.

  LOCAL x IS ROUND(TERMINAL:WIDTH / 2) - FLOOR(s:LENGTH / 2).
  PRINT s AT (x,y).
}

FUNCTION update_setting {
  PARAMETER k,v.
  SET settings[k] TO v.
  WRITEJSON(settings,"1:/config/settings.json").
}

FUNCTION play_sounds{
  PARAMETER s.
  if settings["Sound"] = 1 {
    LOCAL soundpack IS READJSON(vol+"/astar-rover/config/soundpack.json").

    LOCAL q IS LIST().
    LOCAL c IS 0.
    FOR i IN soundpack[s] {
      q:ADD(GetVoice(c)).
      SET q[c]:WAVE TO i[0].
      SET q[c]:ATTACK TO i[1].
      SET q[c]:DECAY TO i[2].
      SET q[c]:SUSTAIN TO 1.
      q[c]:PLAY(i[3]).
      SET c TO c + 1.
    }
  }
}

FUNCTION display_battery
{
  PARAMETER y,c.
  PRINT "################################" AT (6,y).
  PRINT "#" AT (6,y+1).
  PRINT "###" AT (37,y+1).
  PRINT "################################" AT (6,y+2).

    LOCAL p IS ROUND(c / 3.3).
    LOCAL b IS "".
    FROM {local x is 0.} UNTIL x = p STEP {set x to x+1.} DO {
      SET b TO b + "|".
    }
    PRINT b + spc AT (7,y+1).
}

FUNCTION science_menu {
  CLEARSCREEN.
  LOCAL sp IS get_science_parts().
  PRINT "  ---{  Select a science part }---" AT (2,2).
  LOCAL y IS 1.
  FOR p IN sp {
    PRINT "(" + y + ") " + p:TITLE AT (4,y+4).
    SET y TO y + 1.
  }
}

FUNCTION do_science
{
  PARAMETER N.
  LOCAL sp IS get_science_parts().
  LOCAL sc TO sp[N-1].
  LOCAL m IS sc:MODULES.
  FOR o IN m {
    if o = "ModuleScienceExperiment"{
      transmit_science(sc:GETMODULE("ModuleScienceExperiment")).
    } else if o = "DMModuleScienceAnimate" {
      transmit_science(sc:GETMODULE("DMModuleScienceAnimate")).
    }
  }
}

FUNCTION transmit_science
{
  PARAMETER M.

  PRINT "Preparing science part" AT (2,10).
  M:DUMP.
  M:RESET.
  PRINT "Performing Science" AT (2,11).
  WAIT 0.
  M:DEPLOY.

  PRINT "Transmitting Science" AT (2,12).
  WAIT UNTIL M:HASDATA.
  M:TRANSMIT.
  M:RESET.
}

FUNCTION get_science_parts {
  return SHIP:PARTSTAGGED("Skience").

}
