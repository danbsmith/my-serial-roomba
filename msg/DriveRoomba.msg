## Drive at a velocity in mm/s with a turning radius in mm
## Velocity: -500 to 500 mm/s
## Radius: -2000 to 2000 mm
int32 velocity
int32 radius
## The Numbers are 2's complement signed integers.  You remember how those work, right?
## Also, to drive straight, send 32768 (8000) as the radius
## Also also, they are big endian
