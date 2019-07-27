void g2mss(vec &accel){
	accel.x *= 9.81;
	accel.y *= 9.81;
	accel.z *= 9.81;
}

void removeEarthGravity(vec &accel){
	accel.z -= 9.81;
}

void integrate(vec &r, vec a, float samplePeriod){
	r.x += a.x*samplePeriod;
	r.y += a.y*samplePeriod;
	r.z += a.z*samplePeriod;
}
