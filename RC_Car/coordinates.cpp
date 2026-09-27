#include "coordinates.h"
#include <Arduino.h>
#include <math.h>

// Meters per degree of LATITUDE is very close to constant everywhere on
// Earth (~111,320 m). Meters per degree of LONGITUDE shrinks toward the
// poles by a factor of cos(latitude) -- toLocalMeters() applies that
// correction using the origin's latitude. This is the same "~1 meter per
// 0.000009 degrees" scale the .ino's DEG_PER_METER constant approximates
// (1 / 0.000009 = 111,111, matching within ~0.2%) -- kept consistent here
// so calculateCurrDistance() below can keep returning values in the same
// units the .ino already expects and divides by DEG_PER_METER.
const double METERS_PER_DEG_LAT = 111320.0;
const double DEG_PER_METER_APPROX = 0.000009; // must match the .ino's DEG_PER_METER

Path::Path() {
    pointCount = 0;
    currLocation.x = 0.0;
    currLocation.y = 0.0;
    currentHeading = 0.0;
    headingEstablished = false;
    originSet = false;
    originX = 0.0;
    originY = 0.0;
    cosOriginLat = 1.0;
}

// Locks in the local tangent-plane origin the first time this class sees
// any real coordinate (whichever comes first: add_point, setCurrentLocation,
// or updateFromGPS). A no-op every call after that.
void Path::ensureOrigin(double x, double y) {
    if (!originSet) {
        originX = x;
        originY = y;
        const double PI_VAL = acos(-1.0);
        cosOriginLat = cos(y * PI_VAL / 180.0);
        originSet = true;
    }
}

// Converts an absolute decimal-degree lon/lat into local east/north
// meters relative to the origin. Subtracting two nearby doubles like
// -105.0167711 and -105.0167541 is numerically clean (no cancellation
// error) -- what it buys us is that everything downstream (atan2, sqrt)
// now operates on small numbers where a float's full precision is
// actually useful, instead of on a large absolute value where most of
// that precision is wasted on digits shared by every point on the path.
void Path::toLocalMeters(double x, double y, double &outEastMeters, double &outNorthMeters) {
    ensureOrigin(x, y);
    double dLon = x - originX;
    double dLat = y - originY;
    outEastMeters = dLon * METERS_PER_DEG_LAT * cosOriginLat;
    outNorthMeters = dLat * METERS_PER_DEG_LAT;
}

void Path::add_point(double x, double y) {
    // Only add a point if we haven't hit the memory limit
    if (pointCount < MAX_POINTS) {
        ensureOrigin(x, y);
        points[pointCount].x = x;
        points[pointCount].y = y;
        pointCount++;
    }
}

void Path::calculate_distances() {
    const double PI_VAL = acos(-1.0);
    if (pointCount > 0) {
        directions[0].distance = 0;
        directions[0].angle = 0;
    }
    for (int i = 1; i < pointCount; ++i) {
        double prevE, prevN, curE, curN;
        toLocalMeters(points[i-1].x, points[i-1].y, prevE, prevN);
        toLocalMeters(points[i].x, points[i].y, curE, curN);
        double dE = curE - prevE;
        double dN = curN - prevN;
        // Real meters now, computed correctly via the local tangent
        // plane -- previously this ran sqrt/atan2 directly on raw
        // lon/lat degree deltas, which silently mixed two axes with
        // different real-world scale (see METERS_PER_DEG_LAT comment
        // above) and wasn't a physically meaningful distance at all.
        directions[i].distance = sqrt(dE * dE + dN * dN);
        directions[i].angle = atan2(dN, dE) * 180.0 / PI_VAL;
    }
}

// Draws a line from the car's CURRENT location straight to the target
// waypoint (recomputed fresh every call, not a fixed reference line from
// where the leg started), and returns the signed angle between that line
// and the car's actual current heading. Positive means the target is to
// the left of where the car is currently pointed, negative means right.
// The caller uses this to steer the car so it points along that line.
//
// NOTE: this steers toward where the car IS right now. Near a waypoint,
// or whenever the bearing-to-target is swinging faster than the car can
// physically turn, this can cause the car to circle/orbit the point
// instead of converging on it. steering_error_predictive() below fixes
// that by aiming at a projected future position instead.
// Returns signed lateral distance from the current GPS position to the
// waypoint-to-waypoint line for the active leg.
//
// The line runs from points[index-1] -> points[index].
// Sign convention (standard EN-plane cross product):
//   positive = car is LEFT of the line  -> caller should steer RIGHT
//   negative = car is RIGHT of the line -> caller should steer LEFT
//
// This is the GPS equivalent of a line-following robot's left/right sensor
// error. It uses position only; currentHeading is deliberately not involved.
double Path::crossTrackErrorMeters(int index) {
    if (index <= 0 || index >= pointCount) {
        return 0.0;
    }

    double startE, startN, endE, endN, curE, curN;
    toLocalMeters(points[index - 1].x, points[index - 1].y, startE, startN);
    toLocalMeters(points[index].x,     points[index].y,     endE,   endN);
    toLocalMeters(currLocation.x,      currLocation.y,      curE,   curN);

    double legE = endE - startE;
    double legN = endN - startN;
    double legLength = sqrt(legE * legE + legN * legN);

    if (legLength < 0.001) {
        return 0.0;
    }

    // 2-D cross product / leg length gives signed perpendicular distance.
    // Positive is LEFT of the directed path; negative is RIGHT.
    double cross = legE * (curN - startN) - legN * (curE - startE);
    return cross / legLength;
}

double Path::alongTrackDistanceMeters(int index) {
    if (index <= 0 || index >= pointCount) {
        return 0.0;
    }

    double startE, startN, endE, endN, curE, curN;
    toLocalMeters(points[index - 1].x, points[index - 1].y, startE, startN);
    toLocalMeters(points[index].x,     points[index].y,     endE,   endN);
    toLocalMeters(currLocation.x,      currLocation.y,      curE,   curN);

    double legE = endE - startE;
    double legN = endN - startN;
    double legLength = sqrt(legE * legE + legN * legN);
    if (legLength < 0.001) {
        return 0.0;
    }

    // Projection of the current position onto the active line, measured
    // from the line's start point. This is position-only; no car heading
    // or bearing is involved.
    double fromStartE = curE - startE;
    double fromStartN = curN - startN;
    return (fromStartE * legE + fromStartN * legN) / legLength;
}

double Path::steering_error(int index) {
    if (!headingEstablished) {
        return 0.0; // no real heading yet -- drive straight until movement establishes one
    }

    const double PI_VAL = acos(-1.0);
    double curE, curN, tgtE, tgtN;
    toLocalMeters(currLocation.x, currLocation.y, curE, curN);
    toLocalMeters(points[index].x, points[index].y, tgtE, tgtN);
    double bearingToTarget = atan2(tgtN - curN, tgtE - curE) * 180.0 / PI_VAL;

    double error = bearingToTarget - currentHeading;
    while (error > 180.0) error -= 360.0;
    while (error <= -180.0) error += 360.0;

    return error;
}

// PURE-PURSUIT STYLE PREDICTIVE STEERING.
//
// Instead of steering based on the bearing from where the car is RIGHT
// NOW to the waypoint, this projects the car's position forward a short
// distance along its current heading first, then computes the bearing
// from that PREDICTED position to the waypoint. This is the standard
// fix for a car that spins/orbits near a waypoint: by the time a
// steering correction actually takes effect, the car has already moved,
// so correcting for where it currently is is always a step behind. This
// steers for where the car will be instead, which damps out that lag
// and stops it from chasing a bearing that keeps sliding around it.
//
// lookaheadMeters: how far ahead to project. Larger = smoother/more
// stable but less precise near tight turns; smaller = tighter tracking
// but closer to the old behavior (and its circling problem) as it
// approaches zero.
double Path::steering_error_predictive(int index, double lookaheadMeters) {
    if (!headingEstablished) {
        return 0.0; // no real heading yet -- drive straight until movement establishes one
    }

    const double PI_VAL = acos(-1.0);

    double curE, curN, tgtE, tgtN;
    toLocalMeters(currLocation.x, currLocation.y, curE, curN);
    toLocalMeters(points[index].x, points[index].y, tgtE, tgtN);

    double headingRad = currentHeading * PI_VAL / 180.0;
    // Project forward directly in local meters. Previously this had to
    // approximate lookaheadMeters as a degree offset (DEG_PER_METER) and
    // add it onto raw lon/lat before converting back -- now that we're
    // already working in a meters-based local frame, the projection is
    // just a straight meters offset, with no approximation step at all.
    double predictedE = curE + cos(headingRad) * lookaheadMeters;
    double predictedN = curN + sin(headingRad) * lookaheadMeters;

    double bearingToTarget = atan2(tgtN - predictedN, tgtE - predictedE) * 180.0 / PI_VAL;

    double error = bearingToTarget - currentHeading;
    while (error > 180.0) error -= 360.0;
    while (error <= -180.0) error += 360.0;

    return error;
}

void Path::print_path() {
    for (int i = 0; i < pointCount; ++i) {
        Serial.print("Point ");
        Serial.print(i);
        Serial.print(": (");
        Serial.print(points[i].x, 8); // Print to 8 decimal places for GPS accuracy
        Serial.print(", ");
        Serial.print(points[i].y, 8);
        Serial.print(") -> Dist(m): ");
        Serial.print(directions[i].distance, 6);
        Serial.print(", Angle: ");
        Serial.print(directions[i].angle);
        Serial.println(" degrees");
    }
}

int Path::numPoints() {
    return pointCount;
}

// Seeds the heading from a known, fixed real-world starting orientation
// instead of waiting for the car to move before steering has anything
// real to work with. Takes a standard compass bearing (0 = north,
// clockwise positive, e.g. 150 = roughly south-southeast) and converts
// it to this class's internal math-angle convention, where atan2(dy,dx)
// measures counter-clockwise from east.
void Path::setInitialHeading(double compassBearingDegrees) {
    double mathAngle = 90.0 - compassBearingDegrees;
    while (mathAngle > 180.0) mathAngle -= 360.0;
    while (mathAngle <= -180.0) mathAngle += 360.0;

    currentHeading = mathAngle;
    headingEstablished = true;
}

void Path::setCurrentLocation(double x, double y) {
    // Position-only update, no heading logic. Used during startup
    // calibration, before the car is actually navigating and before
    // setInitialHeading() seeds a real heading -- heading tracking for
    // the actual drive happens in updateFromGPS() below.
    ensureOrigin(x, y);
    currLocation.x = x;
    currLocation.y = y;
}

// Blends two headings (degrees) on the unit circle rather than averaging
// the numbers directly, so a heading near the -180/180 wraparound (e.g.
// -179° and 179°, which are only 2° apart in reality) blends to
// something sensible instead of averaging to ~0°, which would be a
// heading pointing the completely wrong way.
double Path::blendHeadings(double oldHeadingDeg, double newHeadingDeg, double alpha) {
    const double PI_VAL = acos(-1.0);
    double oldRad = oldHeadingDeg * PI_VAL / 180.0;
    double newRad = newHeadingDeg * PI_VAL / 180.0;
    double x = (1.0 - alpha) * cos(oldRad) + alpha * cos(newRad);
    double y = (1.0 - alpha) * sin(oldRad) + alpha * sin(newRad);
    return atan2(y, x) * 180.0 / PI_VAL;
}

// Updates position AND heading from a new GPS fix. This replaces the old
// "just diff the last two positions" heading estimate, which was prone
// to reacting to single noisy/glitched fixes as if they were real turns
// -- especially at slow (push-test) speeds, where a 1-2m GPS jitter and
// actual movement are the same order of magnitude.
//
// Heading is only updated when there's a source trustworthy enough to
// use, in priority order:
//   1. GPS-reported course-over-ground (from the RMC sentence), when the
//      module provides it and speed is high enough to trust it. This is
//      computed by the receiver itself and is normally far less noisy
//      than differencing two raw lat/lon fixes.
//   2. Position-differencing (old method), as a fallback when course
//      isn't available, but only above a larger movement threshold than
//      before and only at meaningful speed.
//   3. Neither: keep the last known heading rather than let a jittery
//      fix masquerade as a turn.
//
// Whatever new measurement is taken gets passed through a low-pass
// filter (blendHeadings) rather than applied directly, so a single bad
// fix can only nudge the heading a little instead of snapping to it.
void Path::updateFromGPS(double x, double y, double speedMs, double gpsCourseDeg, bool courseValid) {
    const double MIN_SPEED_FOR_HEADING = 0.3;       // m/s -- below this, both GPS
                                                      // course and position-differencing
                                                      // are too noisy relative to actual
                                                      // movement to trust as a heading
                                                      // measurement. Set for a ~1 m/s
                                                      // hand-push test speed with some
                                                      // margin -- raise this later if the
                                                      // car's real driving speed ends up
                                                      // much higher and 0.3 m/s proves
                                                      // too noisy in practice.
    const double MIN_MOVEMENT_FOR_HEADING_METERS = 2.0; // Now a real, latitude-independent
                                                      // meters threshold (was previously a
                                                      // fixed degree delta that only
                                                      // approximated 2m at one specific
                                                      // latitude) -- raised from ~1m so a
                                                      // single noisy/glitched fix can't
                                                      // masquerade as a real turn when
                                                      // falling back to position-differencing.
    const double HEADING_BLEND_ALPHA = 0.25;         // low-pass filter weight given to
                                                      // each NEW heading measurement.
                                                      // TWEAK: lower (e.g. 0.1) = smoother
                                                      // but slower to react to real turns;
                                                      // higher (e.g. 0.5) = reacts faster
                                                      // but lets more noise through.

    ensureOrigin(x, y);
    double prevE, prevN, newE, newN;
    toLocalMeters(currLocation.x, currLocation.y, prevE, prevN);
    toLocalMeters(x, y, newE, newN);
    double dE = newE - prevE;
    double dN = newN - prevN;
    double movedDist = sqrt(dE * dE + dN * dN); // real meters

    double measuredHeading = currentHeading;
    bool haveMeasurement = false;

    if (speedMs >= MIN_SPEED_FOR_HEADING) {
        if (courseValid) {
            const double PI_VAL = acos(-1.0);
            double mathAngle = 90.0 - gpsCourseDeg; // compass bearing -> this class's math-angle convention
            while (mathAngle > 180.0) mathAngle -= 360.0;
            while (mathAngle <= -180.0) mathAngle += 360.0;
            measuredHeading = mathAngle;
            haveMeasurement = true;
        } else if (movedDist >= MIN_MOVEMENT_FOR_HEADING_METERS) {
            const double PI_VAL = acos(-1.0);
            measuredHeading = atan2(dN, dE) * 180.0 / PI_VAL;
            haveMeasurement = true;
        }
    }
    // else: moving too slowly for any heading source to be trustworthy --
    // keep the last known heading rather than react to jitter/noise

    if (haveMeasurement) {
        if (!headingEstablished) {
            currentHeading = measuredHeading; // first real measurement -- take it directly, nothing to blend with yet
            headingEstablished = true;
        } else {
            currentHeading = blendHeadings(currentHeading, measuredHeading, HEADING_BLEND_ALPHA);
        }
    }

    currLocation.x = x;
    currLocation.y = y;
}

double Path::getCurrentHeading() {
    return currentHeading;
}

Coordinates Path::getCurrentPath(int i){
    return points[i];
}

// Projects currLocation forward along currentHeading by speedMs*elapsedSeconds,
// then converts back to absolute lon/lat. Used to keep the steering
// calculations fed with a reasonable position estimate DURING the gap
// between real GPS fixes, instead of freezing on stale data until the
// next full NMEA sentence has arrived and been parsed. This is pure
// extrapolation, not a measurement -- it can't correct heading, and any
// error it introduces just gets wiped out the next time updateFromGPS()
// runs with a real fix. Caller is responsible for not calling this with
// an implausibly large elapsedSeconds (e.g. after a long GPS dropout),
// since extrapolating too far ahead on a stale heading/speed is worse
// than just holding position.
void Path::predictAhead(double elapsedSeconds, double speedMs) {
    if (!headingEstablished || speedMs <= 0.0 || elapsedSeconds <= 0.0) {
        return; // nothing trustworthy to extrapolate from yet
    }

    const double PI_VAL = acos(-1.0);
    double curE, curN;
    toLocalMeters(currLocation.x, currLocation.y, curE, curN);

    double headingRad = currentHeading * PI_VAL / 180.0;
    double distMeters = speedMs * elapsedSeconds;
    double newE = curE + cos(headingRad) * distMeters;
    double newN = curN + sin(headingRad) * distMeters;

    // Back to absolute lon/lat, inverting the same toLocalMeters() scaling.
    currLocation.x = originX + (newE / (METERS_PER_DEG_LAT * cosOriginLat));
    currLocation.y = originY + (newN / METERS_PER_DEG_LAT);
}

Coordinates Path::getCurrentLocation(){
    Serial.print("Current Location - X: ");
    Serial.print(currLocation.x, 8);
    Serial.print(" Y: ");
    Serial.println(currLocation.y, 8);
    return currLocation;
}

double Path::calculateCurrDistance(int index) {
    if (index < 0 || index >= pointCount) {
        return -1;
    }
    double curE, curN, tgtE, tgtN;
    toLocalMeters(currLocation.x, currLocation.y, curE, curN);
    toLocalMeters(points[index].x, points[index].y, tgtE, tgtN);
    double dE = tgtE - curE;
    double dN = tgtN - curN;
    double realMeters = sqrt(dE * dE + dN * dN);
    // Converted back into the same "pseudo-degree" units the .ino has
    // always expected from this function (it recovers real meters via
    // `/ DEG_PER_METER`) so no caller needs to change. The distance
    // itself is now computed correctly from real local meters instead
    // of a raw degree-space Euclidean distance that silently mixed two
    // axes with different real-world scale.
    return realMeters * DEG_PER_METER_APPROX;
}

double Path::calculateCurrAngle(int index){
    const double PI_VAL = acos(-1.0);
    double curE, curN, tgtE, tgtN;
    toLocalMeters(currLocation.x, currLocation.y, curE, curN);
    toLocalMeters(points[index].x, points[index].y, tgtE, tgtN);
    return atan2(tgtN - curN, tgtE - curE) * 180.0 / PI_VAL;
}