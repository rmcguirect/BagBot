//uint8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length, uint16_t max_loops)  testing removal of length
uint8_t GetCurrentFIFOPacket(uint8_t *data, uint16_t max_loops)
{
  //mpu.resetFIFO();
  //delay(1);

  fifoCount = mpu.getFIFOCount();
  GetPacketLoopCount = 0;

  //11/10/19 I have seen this fail with fifoC > 28 on occasion, so now I loop max three times
  OuterGetPacketLoopCount = 0;
  while (fifoCount != packetSize && OuterGetPacketLoopCount <= 3)
  {
    mpu.resetFIFO();
    delay(1);

    fifoCount = mpu.getFIFOCount();
    GetPacketLoopCount = 0;

    //mySerial.printf("In GetCurrentFIFOPacket: before loop fifoC = %d\t",fifoC);
    while (fifoCount < packetSize && GetPacketLoopCount < max_loops)
    {
      GetPacketLoopCount++;
      fifoCount = mpu.getFIFOCount();
      //delay(2);
    }

    if (GetPacketLoopCount >= max_loops)
    {
      return 0;
    }

    //if we get to here, there should be exactly one packet in the FIFO
    OuterGetPacketLoopCount++;
  }

  if (OuterGetPacketLoopCount < 3)
  {
    mpu.getFIFOBytes(data, packetSize);
    return 1;
  }

  return 0; //failed to get a good packet
}
