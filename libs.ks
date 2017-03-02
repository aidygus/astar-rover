FUNCTION center {
  PARAMETER string,y.

  LOCAL x IS ROUND(TERMINAL:WIDTH / 2) - FLOOR(string:LENGTH / 2).
  PRINT string AT (x,y).
}

FUNCTION play_sounds{
  PARAMETER s.
  LOCAL soundpack IS READJSON("0:/astar-rover/config/soundpack.json").

  LOCAL seq IS LIST().
  LOCAL c IS 0.
  FOR i IN soundpack[s] {
    seq:ADD(GetVoice(c)).
    SET seq[c]:WAVE TO i[0].
    SET seq[c]:ATTACK TO i[1].
    SET seq[c]:DECAY TO i[2].
    SET seq[c]:SUSTAIN TO 1.
    seq[c]:PLAY(i[3]).
    SET c TO c + 1.
  }
}
