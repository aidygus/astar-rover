
SET s TO get_science_parts().
SET runmode TO 0.

until runmode = -1 {
    if runmode = 0 {
      CLEARSCREEN.
      PRINT "  ---{  Select a science part }---" AT (2,2).
      LOCAL y IS 1.
      FOR p IN s {
        PRINT "(" + y + ") " + p:TITLE AT (4,y+4).
        SET y TO y + 1.
      }
      SET runmode TO 1.
    } else if runmode = 1 {
      IF TERMINAL:INPUT:HASCHAR {
        SET K TO TERMINAL:INPUT:GETCHAR().
        SET N TO K:TONUMBER(-99).
        IF N <> -99 {
          SET sc TO s[N-1].
          SET mods TO sc:MODULES.
          FOR mo IN mods {
            if mo = "ModuleScienceExperiment"{
              SET M TO sc:GETMODULE("ModuleScienceExperiment").
            } else if mo = "DMModuleScienceAnimate" {
              SET M TO sc:GETMODULE("DMModuleScienceAnimate").
            }
          }

          M:DUMP.
          M:RESET.
          WAIT 0.1.
          M:DEPLOY.
          WAIT 0.5.
          if  M:HASDATA {
            M:TRANSMIT.
          }
        SET runmode TO 0.
      }
    }
  }
}




FUNCTION get_science_parts {
  return SHIP:PARTSTAGGED("Skience").

}
