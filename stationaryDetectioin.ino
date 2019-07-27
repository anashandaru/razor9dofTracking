bool isStationary(float accelX,float accelY,float accelZ){
	float accel    = sqrt(sq(accelX)+sq(accelY)+sq(accelZ));
	float weighted = lowpassFilter.input(abs(highpassFilter.input(accelY)));
	return weighted<0.03?true:false;
}

vec cross(vec a, vec b){
	vec result;
	result.x = a.y*b.z-a.z*b.y;
	result.y = a.z*b.x-a.x*b.z;
	result.z = a.x*b.y-a.y*b.x;
	return result;
}

float dot(vec a, vec b){
	return (a.x*b.x + a.y*b.y + a.z*b.z);
}

vec mul(float a, vec b){
	b.x *= a;
	b.y *= a;
	b.z *= a;
	return b;
}

vec add(vec a, vec b){
	a.x += b.x;
	a.y += b.y;
	a.z += b.z;
	return a;
}

vec rotateVec(vec v, vec q, float w){
	vec r1 = mul(2.0 * dot(q,v),q);
	vec r2 = mul(sq(w)-dot(q,q),v);
	vec r3 = mul(2.0*w,cross(q,v));
	vec r = add(r1,r2);
	r = add(r1,r2);
	r = add(r,r3);
	return r;
}

vec rotateVec2(vec v, vec q, float w){
	vec t = mul(2, cross(q,v));
	vec r = add(v,mul(w,t));
	r = add(r,cross(q,t));
	return r;
}

quat quatProd(quat a, quat b){
	quat r;
	r.w = a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z;
	r.x = a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y;
	r.y = a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x;
	r.z = a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w;
	return r;
}

quat quatConj(quat q){
	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;
	return q;
}

vec quatRotate(vec v, quat q){
	quat vn, vr;
	vn.x = v.x;
	vn.y = v.y;
	vn.z = v.z;
	vn.w = 0;

	vr = quatProd(quatProd(q,vn),quatConj(q));
	v.x = vr.x;
	v.y = vr.y;
	v.z = vr.z;
	return v;
}
