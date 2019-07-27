bool isStationary(float accelX,float accelY,float accelZ){
	float accel    = sqrt(sq(accelX)+sq(accelY)+sq(accelZ));
	float weighted = lowpassFilter.input(abs(highpassFilter.input(accelY)));
	return weighted<0.03?true:false;
}