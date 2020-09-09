void PWMDrive(int LPin, int RPin, int LDirPin, int RDirPin, int& LVal, int& RVal){
  int LOut=0;
  bool LDir=0;

  int ROut=0;
  bool RDir=1;


  LOut=abs(map(LVal,-130,130,-255,255));
  ROut=abs(map(RVal,-130,130,-255,255));

  if (LVal<0)
  {
    LDir=1;
  }

  if (RVal<0)
  {
    RDir=0;
  }

  analogWrite(LPin,LOut);
  digitalWrite(LDirPin, LDir);
  
  analogWrite(RPin,ROut);
  digitalWrite(RDirPin, RDir);



//  Serial.println("Left Output:");
//  Serial.println(LOut);
//  Serial.println(LDir);
//
//  
//  Serial.println("Right Output:");
//  Serial.println(ROut);
//  Serial.println(RDir);


  
}
