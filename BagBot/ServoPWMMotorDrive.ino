void ServoPWMDrive(Servo LSer, Servo RSer, int& LVal, int& RVal){
  
  int LOut=0;
  int ROut=0;

  LOut=abs(map(LVal,-130,130,0,180));
  //Probably going to have to map this reversed to compensate for reversed motor.
  ROut=abs(map(RVal,-130,130,0,180));

  LSer.write(LVal);
  RSer.write(RVal);

  Serial.println(" ");
  Serial.println("LVal: ");
  Serial.println(LVal);

  
  
}
