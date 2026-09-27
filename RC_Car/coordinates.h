#ifndef PATH_H
#define PATH_H

#include <Arduino.h> // Required for standard Arduino String
#include <stdint.h>

#define MAX_POINTS 16 // Sized for realistic path lengths (currently 9 points
                       // used) with headroom, not the full 50 the original
                       // value reserved. On the Uno's 2KB of total SRAM,
                       // points[] and directions[] at MAX_POINTS=50 were
                       // reserving ~800 bytes for a path that only ever used
                       // ~150 -- a major contributor to running out of RAM
                       // and silently corrupting other variables (see the
                       // "Current Location" fields reading back as exactly
                       // 0.0 with nothing in between that could have written
                       // that). Raise this only as far as your actual
                       // longest planned path needs.

// Positions are held as signed integers in units of 1e-7 degrees
// ("E7"), never as double/float degrees. On AVR, double IS float: a
// 24-bit mantissa can only resolve about 7.6e-6 degrees near longitude
// -105, i.e. a 0.65 m east / 0.43 m north grid. Every fix therefore
// snapped to that grid before any geometry ran, which quantised
// cross-track error into ~0.6 m steps and inverted its sign for true
// offsets under ~0.3 m -- the car steering away from the line instead of
// back onto it. int32 E7 covers the full +-180 degrees with 1.1 cm
// resolution and is exact under subtraction, so the local-meters
// conversion below is the first place a float is used at all.
struct Coordinates {
    int32_t x; // longitude, 1e-7 degrees
    int32_t y; // latitude, 1e-7 degrees
};

// Prints an E7 value as plain decimal degrees (7 decimal places) using
// integer math, so the serial log shows the real resolution of the fix
// instead of float noise padded out to 8 digits.
void printDegreesE7(int32_t valueE7);

struct Direction {
    double distance; // meters -- see calculate_distances()
    double angle;    // degrees
};

class Path {
public:
    Path(); // Constructor to set up the arrays
    void add_point(int32_t lonE7, int32_t latE7);
    void calculate_distances();
    double steering_error(int index);
    double steering_error_predictive(int index, double lookaheadMeters);
    double crossTrackErrorMeters(int index);
    double alongTrackDistanceMeters(int index);
    void print_path();
    int numPoints();
    void setCurrentLocation(int32_t lonE7, int32_t latE7);
    void updateFromGPS(int32_t lonE7, int32_t latE7, double speedMs, double gpsCourseDeg, bool courseValid);
    void setInitialHeading(double compassBearingDegrees);
    Coordinates getCurrentLocation();
    double getCurrentHeading();
    double calculateCurrDistance(int index); // meters
    double calculateCurrAngle(int index);
    Coordinates getCurrentPath(int i);
    void predictAhead(double elapsedSeconds, double speedMs);
    // DEAD RECKONING: extrapolates currLocation forward using the last
    // known heading and speed, for the time between real GPS fixes.
    // Doesn't touch currentHeading -- only position. A real fix (via
    // updateFromGPS) always overwrites this with ground truth, so any
    // drift this introduces is self-correcting and bounded by how long
    // it's actually been since the last real fix.

private:
    Coordinates points[MAX_POINTS];
    Direction directions[MAX_POINTS];
    int pointCount;
    Coordinates currLocation;
    double currentHeading;   // degrees, filtered estimate of actual direction of travel
    bool headingEstablished; // false until a trustworthy heading measurement has been taken

    // --- LOCAL TANGENT PLANE ---
    // Geometry runs in EAST/NORTH meters relative to a fixed origin (the
    // first point this class ever sees), so a float's precision budget
    // goes to the few hundred meters of real variation along the path
    // rather than the unchanging ~105-degree offset shared by every
    // point. The degree -> meter conversion also scales longitude by
    // cos(latitude), which a single DEG_PER_METER constant did not: a
    // degree of longitude is ~23% shorter than a degree of latitude at
    // this car's ~40N starting point.
    bool originSet;
    int32_t originX;      // longitude of the origin, 1e-7 degrees
    int32_t originY;      // latitude of the origin, 1e-7 degrees
    double cosOriginLat;  // cached cos(origin latitude in radians), for longitude scaling

    void ensureOrigin(int32_t lonE7, int32_t latE7);
    void toLocalMeters(int32_t lonE7, int32_t latE7, double &outEastMeters, double &outNorthMeters);
    int32_t lonE7FromEastMeters(double eastMeters);
    int32_t latE7FromNorthMeters(double northMeters);
    double blendHeadings(double oldHeadingDeg, double newHeadingDeg, double alpha);
};

#endif