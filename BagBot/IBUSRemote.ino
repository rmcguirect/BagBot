void RadioRead(){
  //This is where ibus data is read from the serial port.
  ch1=IBus.readChannel(0); // get latest value for servo channel 1
  ch2=IBus.readChannel(1); // get latest value for servo channel 2
  ch3=IBus.readChannel(2); // get latest value for servo channel 2
  ch4=IBus.readChannel(3); // get latest value for servo channel 2
  ch5=IBus.readChannel(4); // get latest value for servo channel 5
  ch6=IBus.readChannel(5); // get latest value for servo channel 6
  ch7=IBus.readChannel(6); // get latest value for servo channel 7
  ch8=IBus.readChannel(7); // get latest value for servo channel 8
}
