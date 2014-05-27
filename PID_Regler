#include <Arduino.h>
 
 
class PID{
 
 
public:
 
PID( float *ncf, float ng, int8_t nl,float nh);  // coeffs, gain , order, steplength
 
void setCoeffs(float * cf); // change the coeffcient array
 
void setIerr(float);
 
...
 
void reset(void); // set all internal values to zero
 
void setOutputLimits(float,float);
 
 
float *y; // last outputs
float *e; // last errors
 
float step(float,float);  // next timestep
 
private:
float gain; //overall gain
float* cf;  // coefficients
float h ; // time between samples
 
float ierr; // integrated error
...
 
};
