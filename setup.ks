RUNPATH("0:/astar-rover/libs.ks").

SET rversion TO OPEN("0:/astar-rover/config/version"):READALL:STRING. //"2.3.1".

LOCAL semode IS 0.
// Valid semodes
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
    "TurnLimit", LIST(0.1,0.1,3),
    "Sound",LIST(1,0,1)
).

if EXISTS("1:/config/settings.json") = FALSE {
  if EXISTS("0:/astar-rover/backup/" + SHIP:NAME + ".json") {
    SET settings TO READJSON("0:/astar-rover/backup/" + SHIP:NAME + ".json").
  } else {
    SET settings TO LEXICON(
      "MinSlope", -15,
      "MaxSlope", 25,
      "IPU", 2000,
      "DefaultSpeed", 4,
      "TurnLimit", 0.2,
      "Sound", 1,
      "Odometer",0,
      "Version",rversion,
      "SendScience",True
    ).
  }
} else {
  SET settings TO READJSON("1:/config/settings.json").
  SET settings["Version"] TO rversion.
  WRITEJSON(settings,"1:/config/settings.json").
}
main_hud().

SET row TO "----------------------------------------------".

UNTIL semode = -1 {
  PRINT "Runmode : " + semode AT (2,TERMINAL:HEIGHT-2).
  IF TERMINAL:INPUT:HASCHAR {
    LOCAL K IS TERMINAL:INPUT:GETCHAR().
    LOCAL N IS K:TONUMBER(-99).
    IF semode = 1 {
      SET semode TO 0.
      main_hud().
    }
    IF K = TERMINAL:INPUT:ENDCURSOR {
      SET semode TO -1.
      CLEARSCREEN.
    } else IF K = TERMINAL:INPUT:HOMECURSOR {
      SET semode TO 0.
      main_hud().
    } else if (semode = 2) {
      handler_settings(K,N).
    } ELSE IF N <> -99 {
      if semode = 0 {
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
  PRINT "(4) Backup Settings to Archive" AT (5,10).
  if EXISTS("0:/astar-rover/backup/" + SHIP:NAME + ".json") {
    PRINT "(5) Restore Settings from Archive" AT (5,11).
  }
  PRINT "(9) Reset to Factory Settings" AT (5,15).
  PRINT "Press the Home key to return to this menu" AT (5,20).
  PRINT "Press End key to exit" AT (5,22).

}

FUNCTION handler_hud {
  PARAMETER N.
  SET semode TO N.
  if N = 1 {
    initiate().
  } else if N = 2 {
    settings_hud().
  } else if N = 3 {
    REBOOT.
  } else if N = 4 {
    COPYPATH("1:/config/settings.json","0:/astar-rover/backup/"+SHIP:NAME+".json").
  } else if N = 5 {
    COPYPATH("0:/astar-rover/backup/"+SHIP:NAME+".json","1:/config/settings.json").
    REBOOT.
  } else if N = 6 {
    runpath("0:/astar-rover/CalcCE.ks").
  } else if N = 9 {
    reset().
  } else {
    SET semode TO 0.
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
  PRINT "| 6 | Enable Sound    |                      |" AT (2,13).
  PRINT row AT (2,12).

  PRINT "Press number of value you wish to edit." AT (2,16).
  PRINT "Use the Up and Down cursor arrows to" AT (2,17).
  PRINT "select values" AT ( 3,18).
  PRINT "Settings will automatically save" AT (2,19).
  PRINT settings["MinSlope"] AT (28,3).
  PRINT settings["MaxSlope"] AT (28,5).
  PRINT settings["IPU"] AT (28,7).
  PRINT settings["DefaultSpeed"] AT (28,9).
  PRINT settings["TurnLimit"] AT (28,11).
  PRINT settings["Sound"] AT (28,13).
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
    // report("Compiling A* scripts",5,y).
    // COMPILE "0:/astar-rover/astar.ks" TO "1:/astar-rover/astar.ksm".
    // SET Y TO report(" - Done.",30,y).
  }
  SET y TO report("Preparing boot directory",2,y).
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
    WRITEJSON(settings,"1:/config/settings.json").
    COPYPATH("0:/astar-rover/config/version","1:/config/version").
  }
  if capacity >= 30000 {
    report("Compiling shared libs",5,y).
    COMPILE "0:/astar-rover/libs.ks" TO "1:/astar-rover/libs.ksm".
    SET Y TO report(" - Done.",30,y).
    // report("Compiling setup",5,y).
    // COMPILE "0:/astar-rover/setup.ks" TO "1:/astar-rover/setup.ksm".
    // SET Y TO report(" - Done.",30,y).
    report("Copying Soundpack",5,y).
    COPYPATH("0:/astar-rover/config/soundpack.json","1:/astar-rover/config/soundpack.json").
    SET Y TO report(" - Done.",30,y).
  }
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

FUNCTION handler_settings {
  PARAMETER K,N.
  LOCAL set IS LIST(3,5,7,9,11,13).
  LOCAL keys IS settings:KEYS.
  IF N <> -99 {
    FOR s IN set {
      PRINT " " AT (26,s).
    }
    if N < set:LENGTH AND N <> 0 {
      PRINT "*" AT (26,set[N-1]).
      SET value TO N.
    }
  } else {
    IF value <> -1 {
      IF K = TERMINAL:INPUT:UPCURSORONE {
        PRINT "Up  " + value AT (28,TERMINAL:HEIGHT-1).
        IF settings[keys[value-1]] < default_values[keys[value-1]][2] {
          update_setting(keys[value-1],settings[keys[value-1]] + default_values[keys[value-1]][0]).
        }
      }
      ELSE IF K = TERMINAL:INPUT:DOWNCURSORONE {
        PRINT "Down" + value AT (28,TERMINAL:HEIGHT-1).
        IF settings[keys[value-1]] > default_values[keys[value-1]][1] {
          update_setting(keys[value-1],settings[keys[value-1]] - default_values[keys[value-1]][0]).
        }
      }
      SET settings TO READJSON("1:/config/settings.json").
      PRINT settings[keys[value-1]]+"    " AT (28,set[value-1]).
    }
  }
}

FUNCTION reset {
  CLEARSCREEN.
  center("---{ Resetting rover to factory settings }---",2).
  DELETEPATH("1:/boot").
  DELETEPATH("1:/astar-rover").
  DELETEPATH("1:/config").
  report("Rover has been reset",2,4).
  report("Press Home key to continue",2,5).
  SET semode TO 0.
}
