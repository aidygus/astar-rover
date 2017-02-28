LOCAL runmode IS 0.

// Valid runmodes
//  0 - Main Menu
//  1 - Automated installation
//  2 - Settings Menu
// 10 - Change Min slope
// 11 - Change Max slope
// 12 - Change IPU
// 13 - Change DefaultSpeed

CLEARSCREEN.

LOCAL settings IS LEXICON().
LOCAL error IS "".
LOCAL value IS -1.
LOCAL default_values IS LEXICON(
    "MinSlope", LIST(1,-45,0),
    "MaxSlope", LIST(1,0,45),
    "IPU", LIST(500,500,2000),
    "DefaultSpeed", LIST(1,1,50),
    "TurnLimit", LIST(0.01,0.01,2.0)
).

if EXISTS("1:/config/settings.json") = FALSE {
  SET settings TO LEXICON(
    "MinSlope", -15,
    "MaxSlope", 25,
    "IPU", 2000,
    "DefaultSpeed", 4,
    "TurnLimit", 0.2
  ).
  WRITEJSON(settings,"1:/config/settings.json").
} else {
  SET settings TO READJSON("1:/config/settings.json").
}
main_hud().

SET row TO "----------------------------------------------".

UNTIL runmode = -1 {
  PRINT "Runmode : " + runmode AT (2,TERMINAL:HEIGHT-2).
  IF TERMINAL:INPUT:HASCHAR {
    LOCAL K IS TERMINAL:INPUT:GETCHAR().
    LOCAL N IS K:TONUMBER(-99).
    IF runmode = 1 {
      SET runmode TO 0.
      main_hud().
    }
    IF K = TERMINAL:INPUT:ENDCURSOR {
      SET runmode TO -1.
      CLEARSCREEN.
    } else IF K = TERMINAL:INPUT:HOMECURSOR {
      SET runmode TO 0.
      main_hud().
    } else if (runmode = 2) {
      handler_settings(K,N).
    } ELSE IF N <> -99 {
      if runmode = 0 {
          handler_hud(N).
      }
    }
    PRINT "Key " + K + " pressed" AT (2,TERMINAL:HEIGHT-1).
  }
  WAIT 0.
}

FUNCTION main_hud {
  CLEARSCREEN.
  center("---{ A* Rover }---",2).
  center("Pick an option below to configure and set up the rover",4).
  PRINT "(1) Initialize rover" AT (5,7).
  PRINT "(2) Rover Settings" AT (5,8).
  PRINT "(3) Reboot KOS Processor" AT (5,9).
  PRINT "(9) Reset to Factory Settings" AT (5,13).
  PRINT "Press the Home key to return to this menu" AT (5,18).
  PRINT "Press End key to exit" AT (5,19).

}

FUNCTION handler_hud {
  PARAMETER N.
  SET runmode TO N.
  if N = 1 {
    initiate().
  } else if N = 2 {
    settings_hud().
  } else if N = 3 {
    REBOOT.
  } else if N = 9 {
    reset().
  } else {
    SET runmode TO 0.
  }
}

FUNCTION settings_hud {
  CLEARSCREEN.
  PRINT row AT (2,2).
  PRINT "| 1 | Min Slope       |                      |" AT (2,3).
  PRINT row AT (2,4).
  PRINT "| 2 | Max Slope       |                      |" AT (2,5).
  PRINT row AT (2,6).
  PRINT "| 3 | IPU             |                      |" AT (2,7).
  PRINT row AT (2,8).
  PRINT "| 4 | Default Speed   |                      |" AT (2,9).
  PRINT row AT (2,10).
  PRINT "| 5 | Turn Limit      |                      |" AT (2,11).
  PRINT row AT (2,12).

  PRINT "Press number of value you wish to edit." AT (2,14).
  PRINT "Use the Up and Down cursor arrows to" AT (2,15).
  PRINT "select values" AT ( 3,16).
  PRINT "Settings will automatically save" AT (2,17).
  PRINT settings["MinSlope"] AT (28,3).
  PRINT settings["MaxSlope"] AT (28,5).
  PRINT settings["IPU"] AT (28,7).
  PRINT settings["DefaultSpeed"] AT (28,9).
  PRINT settings["TurnLimit"] AT (28,11).
}

FUNCTION initiate {
  CLEARSCREEN.
  LOCAL y IS 4.
  center("---{ Initializing your rover }---",2).
  SET y TO report("Switching to rover's local Volume",2,y).
  SWITCH TO 1.
  LOCAL capacity IS CORE:CURRENTVOLUME:CAPACITY.
  SET y TO report("Volume " + CORE:CURRENTVOLUME:NAME +" has a capacity of " + ROUND(capacity/1000) + " kbytes ",2,y).
  if capacity < 15000 {
    SET y TO report("15 kbytes is required to run scripts localy.   - Skipping",5,y).
  } else {
    SET y TO report("Checking if asrover directory exists",2,y).
    IF EXISTS("1:/astar-rover") = FALSE {
      SET y TO report(" - Creating asrover directory.",6,y).
      CREATEDIR("1:/astar-rover").
    }
    report("Compiling rover script",5,y).
    COMPILE "0:/astar-rover/rover.ks" TO "1:/astar-rover/rover.ksm".
    SET y TO report(" - Done.",30,y).
    report("Compiling A* scripts",5,y).
    COMPILE "0:/astar-rover/astar.ks" TO "1:/astar-rover/astar.ksm".
    SET Y TO report(" - Done.",30,y).
    PRINT ROUND(CORE:CURRENTVOLUME:FREESPACE/1000,3) + " kbytes free" AT (5,y).
  }
  SET y TO report("Checking if boot directory exists",2,y).
  IF EXISTS("1:/boot") {
    DELETEPATH("1:/boot").
  }
  SET y TO report(" - Creating boot directory.",6,y).
  CREATEDIR("1:/boot").
  SET y TO report(" - Copying boot file.",6,y).
  COPYPATH("0:/astar-rover/config/runrover.ks","1:/boot/runrover.ks").
  SET y TO report(" - Setting boot file to run at startup.",6,y).
  SET CORE:PART:GETMODULE("kOSProcessor"):BOOTFILENAME TO "/boot/runrover.ks".

  SET y TO report("Checking if config directory exists",2,y).
  IF EXISTS("1:/config") = FALSE {
    SET y TO report(" - Creating config directory.",5,y).
    CREATEDIR("1:/config").
  }
  SET y TO report("Initialization process completed ",2,y+2).
  SET y TO report(ROUND(CORE:CURRENTVOLUME:FREESPACE/1000,3) + " kbytes free",5,y).
  SET y TO report("To customize how the rover operates, change them in the settings menu",2,y+2).
  SET y TO report("Press Any key to continue",5,y+4).
}

FUNCTION report {
  PARAMETER string, x, y.
  PRINT string AT (x,y).
  WAIT 0.2.
  SET y TO y + 1.
  return y.
}

FUNCTION center {
  PARAMETER string,y.

  LOCAL x IS ROUND(TERMINAL:WIDTH / 2) - FLOOR(string:LENGTH / 2).
  PRINT string AT (x,y).
}

FUNCTION handler_settings {
  PARAMETER K,N.
  LOCAL set IS LIST(3,5,7,9,11).
  LOCAL keys IS settings:KEYS.
  IF N <> -99 {
    FOR s IN set {
      PRINT " " AT (26,s).
    }
    PRINT "*" AT (26,set[N-1]).
    SET value TO N.
  } else {
    IF value <> -1 {
      IF K = TERMINAL:INPUT:UPCURSORONE {
        IF settings[keys[value-1]] < default_values[keys[value-1]][2] {
          SET settings[keys[value-1]] TO settings[keys[value-1]] + default_values[keys[value-1]][0].
        }
      }
      ELSE IF K = TERMINAL:INPUT:DOWNCURSORONE {
        IF settings[keys[value-1]] > default_values[keys[value-1]][1] {
          SET settings[keys[value-1]] TO settings[keys[value-1]] - default_values[keys[value-1]][0].
        }
      }
      PRINT settings[keys[value-1]]+"    " AT (28,set[value-1]).
      WRITEJSON(settings,"1:/config/settings.json").
    }
  }
}

FUNCTION reset {
  CLEARSCREEN.
  center("---{ Resetting rover to factory settings }---",2).
  DELETEPATH("1:/boot").
  DELETEPATH("1:/asrover").
  DELETEPATH("1:/config").
  report("Rover has been reset",2,4).
  report("Press Home key to continue",2,5).
  SET runmode TO 0.
}
