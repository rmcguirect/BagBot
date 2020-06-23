void PWMDrive(int LPin, int RPin, int LDirPin, int RDirPin, int& LVal, int& RVal){
  int LOut=0;
  bool LDir=1;

  int ROut=0;
  bool RDir=1;


  LOut=abs(map(LVal,-127,127,-255,255));
  ROut=abs(map(RVal,-127,127,-255,255));

  if (LVal<0)
  {
    LDir=0;
  }

  if (RVal<0)
  {
    RDir=0;
  }

  analogWrite(LPin,LOut);
  analogWrite(RPin,ROut);

  digitalWrite(LDirPin, LDir);
  digitalWrite(RDirPin, RDir);
  
}
