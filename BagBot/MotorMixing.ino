void mixing(int nJoyX,int nJoyY, int& leftM, int& rightM) {
  /*
   * Mixes the throttle and steering for differential steering.
   * Inputs:
   * Throttle - to 127
   * Steering -127 to 127
   * Outputs:
   *Left Motor -127 to 127
   *Right Motor -127 to 127
   */

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 32.0;
   
  // OUTPUTS
  int     nMotMixL;           // Motor (left)  mixed output           (-127..+127)
  int     nMotMixR;           // Motor (right) mixed output           (-127..+127)

  // TEMP VARIABLES
  float   nMotPremixL;    // Motor (left)  premixed output        (-127..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-127..+127)
  int     nPivSpeed;      // Pivot Speed                          (-127..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )

   

  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
  }
  
  
  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY/127.0;
  nMotPremixR = nMotPremixR * nJoyY/127.0;
  
  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
  nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);
  leftM=nMotMixL;
  rightM=nMotMixR;
    
}
