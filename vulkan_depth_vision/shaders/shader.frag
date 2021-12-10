

#version 450
#extension GL_ARB_separate_shader_objects : enable


layout(location = 0) in vec3 fragColor;

layout(location = 0) out vec4 outColor;



float interpolate( float val, float y0, float x0, float y1, float x1 ) {
	return (val-x0)*(y1-y0)/(x1-x0) + y0;
}

float base( float val ) {
	if ( val <= -0.75 ) return 0;
	else if ( val <= -0.25 ) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
	else if ( val <= 0.25 ) return 1.0;
	else if ( val <= 0.75 ) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
	else return 0.0;
}

float red( float gray ) {
	return base( gray - 0.5 );
}
float green( float gray ) {
	return base( gray );
}
float blue( float gray ) {
	return base( gray + 0.5 );
}

void main() {
    float near = 0.01;
    float far = 5.0;

    float d = gl_FragCoord.z / gl_FragCoord.w;
    if(d > 100.f){
        discard;
    }
    d = d/10.;
    d = clamp(d,0,1);
    d =  near * far / (far + gl_FragCoord.z * (near - far));
    d = d/5.0f;
    outColor = vec4(fragColor, 1.0);
    outColor.x=red(d);
    outColor.y=green(d);
    outColor.z=blue(d);


}
