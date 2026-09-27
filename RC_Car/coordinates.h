#ifndef PATH_H
#define PATH_H

#include <Arduino.h> // Required for standard Arduino String

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

struct Coordinates {
    double x; // longitude, decimal degrees
    double y; // latitude, decimal degrees
};

struct Direction {
    double distance; // meters -- see calculate_distances()
    double angle;    // degrees
};

class Path {
public:
    Path(); // Constructor to set up the arrays
    void add_point(double x, double y);
    void calculate_distances();
    double steering_error(int index);
    double steering_error_predictive(int index, double lookaheadMeters);
    double crossTrackErrorMeters(int index);
    double alongTrackDistanceMeters(int index);
    void print_path();
    int numPoints();
    void setCurrentLocation(double x, double y);
    void updateFromGPS(double x, double y, double speedMs, double gpsCourseDeg, bool courseValid);
    void setInitialHeading(double compassBearingDegrees);
    Coordinates getCurrentLocation();
    double getCurrentHeading();
    double calculateCurrDistance(int index); // returns the same "pseudo-degree"
                                              // units as before (caller recovers
                                              // real meters via / DEG_PER_METER,
                                              // as the .ino already does) -- only
                                              // the internal computation changed
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

    // --- LOCAL TANGENT-PLANE PRECISION FIX ---
    // Every point above is still stored as an absolute decimal-degree
    // lon/lat (e.g. -105.0167711), same as before, so nothing outside
    // this class needs to change. But doing trig (atan2/sqrt) DIRECTLY
    // on absolute values like that wastes most of a float's ~7
    // significant digits on the leading digits ("-105.0...") that are
    // identical for every point on a path this small and carry zero
    // steering-relevant information -- only the last couple of digits
    // actually vary between waypoints.
    //
    // Every geometry calculation below instead first subtracts a fixed
    // origin (the first point this class ever sees, via ensureOrigin())
    // and converts to local EAST/NORTH meters via toLocalMeters(). That
    // puts the float's whole precision budget toward the few hundred
    // meters of actual variation in this path instead of an unchanging
    // ~105-degree offset, and it fixes a separate bug where a single
    // DEG_PER_METER constant was implicitly treating 1 degree of
    // longitude as the same distance as 1 degree of latitude -- they
    // aren't equal except at the equator (roughly 15% shorter at ~40°N,
    // where this car's KNOWN_START_LAT sits).
    bool originSet;
    double originX;       // longitude of the origin, decimal degrees
    double originY;       // latitude of the origin, decimal degrees
    double cosOriginLat;  // cached cos(originY in radians), for longitude scaling

    void ensureOrigin(double x, double y);
    void toLocalMeters(double x, double y, double &outEastMeters, double &outNorthMeters);
    double blendHeadings(double oldHeadingDeg, double newHeadingDeg, double alpha);
};

#endif