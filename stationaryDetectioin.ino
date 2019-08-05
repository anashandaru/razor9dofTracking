bool isStationary(float accelX,float accelY,float accelZ){
	float accel    = sqrt(sq(accelX)+sq(accelY)+sq(accelZ));
  float highpassed = abs(highpassFilter.input(accel));
	float weighted = lowpassFilter.input(highpassed);
  SerialPort.print(accel+2);
  SerialPort.print(", ");
  SerialPort.print(weighted+2);
  SerialPort.print(", ");
  SerialPort.print((weighted<StadeThreshold?1:0)+2);
  SerialPort.print(", ");
	return weighted<StadeThreshold?true:false;
}

bool isStationary2(float accelX,float accelY,float accelZ){
  float accel    = sqrt(sq(accelX)+sq(accelY)+sq(accelZ));
  accel -= 1;
  accel = abs(accel);
  float weighted = lowpassFilter.input(accel);
  SerialPort.print(accel+2);
  SerialPort.print(", ");
  SerialPort.print(weighted+2);
  SerialPort.print(", ");
  SerialPort.print((weighted<StadeThreshold?1:0)+2);
  SerialPort.print(", ");
  return weighted<StadeThreshold?true:false;
}

void turnLed(bool ledState){
      digitalWrite(ledPin, ledState);
}
