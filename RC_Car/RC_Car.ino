#include <Servo.h> 
#include <Wire.h>
#include <math.h>
#include "coordinates.h"

// The SparkFun u-blox library is far too large for the Uno's 32KB flash
// (comes in ~10KB over budget even in its reduced mode), since it covers
// dozens of message types we don't need. Instead we talk to the module's
// I2C ("DDC") interface directly with plain Wire calls, and reuse the
// same lightweight NMEA parser that worked with the old serial module --
// only the byte source changes, not the parsing logic.
const uint8_t GPS_I2C_ADDR = 0x42; // u-blox default I2C address

// --- NMEA parsing buffer ---
// Fixed-size char buffer instead of String: avoids heap fragmentation from
// repeated String allocation/deallocation, which can silently hang an Uno
// over time as free RAM is only 2048 bytes total.
#define NMEA_BUFFER_SIZE 90
char nmeaBuffer[NMEA_BUFFER_SIZE];
uint8_t nmeaIndex = 0;

bool birdDetected = false;

// Navigation Data
// Position is carried as int32 in units of 1e-7 degrees ("E7") -- see the
// Coordinates comment in coordinates.h for why a double/float can't hold a
// usable fix on this board.
int32_t xValE7 = 0;
int32_t yValE7 = 0;
double currentSpeedMs = 0.0; // Updated by GPS 

// GPS Fix Quality Data
int satellitesInUse = 0;
double hdop = 99.9; // lower is better; >5 is poor

// --- GPS-LOCK GATE ---
// Nothing drives (calibration, heading seeding, or navigation) until the
// fix quality clears both of these. Without this, the module's very
// first few fixes right after power-on are often a low-satellite/high-HDOP
// rough estimate that hasn't converged yet -- reacting to that as if it
// were real position data is what caused the car to lurch/steer left
// immediately on boot before the fix had actually settled.
const int MIN_SATELLITES_FOR_LOCK = 6;
const double MAX_HDOP_FOR_LOCK = 2.0;
// Raise MIN_SATELLITES_FOR_LOCK / lower MAX_HDOP_FOR_LOCK for a stricter
// (slower to start, more trustworthy) lock. Loosen if it's taking too
// long to lock somewhere with a limited sky view.

// --- KNOWN STARTING LOCATION CALIBRATION ---
// This device always starts from roughly the same real-world spot.
// On boot, we average several fixes, compare that average against the
// known true coordinates below, and apply the resulting offset to every
// reading for the rest of the session. This cancels out the systematic
// bias that varies from power-cycle to power-cycle (the cause of whole
// sessions coming out shifted relative to each other) rather than just
// hoping the receiver "settles" to the right answer on its own.
const int32_t KNOWN_START_LON_E7 = -1050167541L;
const int32_t KNOWN_START_LAT_E7 = 399708443L;
int32_t calibOffsetLonE7 = 0;
int32_t calibOffsetLatE7 = 0;

Path path;

Servo motorDriver;  
Servo myservo;  

// --- TUNING PARAMETERS ---
const int NEUTRAL_THROTTLE = 1500;
const int MAX_THROTTLE = 1650;   // Keep this low during initial testing!
const int MIN_THROTTLE = 1520;   // Minimum throttle to actually move the car
const double TARGET_SPEED = 0.7; // Desired speed in meters/second
const double MAX_PLAUSIBLE_SPEED = 15.0; // Sanity ceiling - reject anything above this as corrupted data

int driveValue = NEUTRAL_THROTTLE; 

// Steering servo:
// Mechanical limits of the steering servo. 95 is straight.
const int STR_MIN = 70;
const int STR_CENTER = 95;
const int STR_MAX = 130;

// MEASURED ON THIS CAR: higher Servo.write() values turn RIGHT, lower
// values turn LEFT. Confirmed from a run where the navigation geometry
// correctly called for a right turn (leg bearing ~200 deg vs heading
// ~144 deg), commanded 70, and the car turned left.
//
// Keep these constants as physical directions, not mathematical signs:
// everywhere below steers "towards STR_RIGHT" or "towards STR_LEFT"
// rather than adding or subtracting raw servo numbers, so this is the
// only place the polarity is encoded.
const int STR_RIGHT = STR_MAX;
const int STR_LEFT = STR_MIN;
const int STR_RIGHT_SIGN = (STR_RIGHT > STR_CENTER) ? 1 : -1;
int currentAngle = STR_CENTER;
double commandedAngle = STR_CENTER; // sub-degree ramp state, see updateSteering()
unsigned long lastSteeringUpdateMillis = 0;

const int escPin = 6; 
const int servoPin = 3;
int32_t startXE7 = 0;
int32_t startYE7 = 0;

// --- GPS LINE-FOLLOWING STEERING ---
// Positive cross-track error means the car is LEFT of the directed waypoint
// line, so the correction is a small RIGHT steering command. Negative means
// RIGHT of the line, so steer LEFT.
double CROSS_TRACK_KP = 8.0; // servo degrees per meter of lateral error
const double CROSS_TRACK_DEADBAND_METERS = 0.15;
const double MAX_CROSS_TRACK_CORRECTION_DEG = 15.0;

// Lateral error is only half the picture: it is exactly zero whenever the
// car sits ON the line, no matter which way it is pointed. Starting a leg
// at the waypoint with the car aimed 50 degrees off it therefore produced
// no steering command at all -- the car drove straight out of the safety
// corridor and stopped. This term steers on the angle between the car's
// heading and the leg direction, so the two together behave like a
// standard line follower: heading gets it pointed down the line,
// cross-track keeps it on it.
const double HEADING_KP = 0.5;              // servo degrees per degree of heading error
const double HEADING_DEADBAND_DEG = 4.0;
const double MAX_HEADING_CORRECTION_DEG = 25.0;

// Cross-track alone cannot start a turn at a waypoint because the car is
// exactly on the new line at the instant the waypoint is reached, making
// cross-track error zero. These settings provide a short, position-only
// corner transition using the geometry of the NEXT line. No GPS heading,
// bearing-to-target, or pure pursuit is used.
const double CORNER_TRANSITION_DISTANCE_METERS = 1.0;
const unsigned long CORNER_TRANSITION_TIMEOUT_MS = 3000;
const double CORNER_TURN_GAIN = 0.22; // servo degrees per degree of path turn
const double MAX_CORNER_INIT_CORRECTION_DEG = 15.0;
const int CORNER_TRANSITION_MAX_THROTTLE = 1540;

// Safety timeout: no fresh GPS fix -> neutral throttle.
const unsigned long GPS_STALE_TIMEOUT_MS = 750;

// Safety corridor around the active waypoint leg. Wide enough that a
// normal correction arc plus a couple of meters of HDOP-1.7 GPS noise
// doesn't trip it, but still well short of "driving at a wall".
const double MAX_ALLOWED_CROSS_TRACK_ERROR_METERS = 3.0;

// First GPS-controlled waypoint on concrete gets half of the original
// available throttle range above neutral: 1500 + (1650-1500)/2 = 1575.
const int FIRST_GPS_WAYPOINT_MAX_THROTTLE = 1575;

// --- STEERING TUNING (adjust these to change how aggressively the car corrects) ---
//
// Think of this like a line-following robot: the "line" is the bearing
// from where the car is right now to the next waypoint. steering_error()
// (in coordinates.cpp) reports how far the car's current heading has
// drifted off that line, in degrees -- exactly like a line-follower's
// sensor reporting how far off the black line it is. Everything below
// turns that drift reading into an actual steering command.

double STEERING_KP = 0.45;   // <-- MAIN KNOB. Scales raw heading error before
                             // anything else happens. Lower = gentler/more
                             // sluggish corrections, higher = snappier but
                             // more prone to overshoot/oscillation.
                             // Tuning procedure: start low (e.g. 0.3-0.4),
                             // raise it in small steps until you see the
                             // car wobble/overshoot around the line, then
                             // back off about 20% from that point.

const double MAX_STEERING_ERROR_DEG = 45.0;
// Heading error (AFTER Kp scaling) at which the servo hits full lock.
// This is separate from Kp on purpose: Kp controls sensitivity to a given
// error, this controls how much error it takes to run out of steering
// angle. Lower this to reach full lock sooner for the same Kp; raise it
// to require a bigger error before maxing out.

const double STEERING_DEADBAND_DEG = 2.0;
// Errors smaller than this are treated as "on the line" and ignored.
// Same trick line-following robots use near dead center -- prevents the
// servo from jittering back and forth when the car is basically already
// pointed the right way. Raise this if you see twitchy micro-corrections.

double LOOKAHEAD_METERS = 2.5;
// PREDICTIVE STEERING (pure pursuit). Instead of steering toward the
// waypoint from where the car IS right now, we project the car forward
// this many meters along its current heading first, then steer toward
// the waypoint from THAT predicted point. This is what fixes
// spinning/orbiting near a waypoint: aiming at where you are right now
// always lags behind where you'll actually be by the time the
// correction takes effect, and the bearing to a close target can swing
// faster than the car can physically turn -- projecting forward removes
// that lag.
// Tuning: too SMALL (near 0) behaves like the old current-position-only
// steering and can still circle. Too LARGE makes the car "cut corners"
// and steer well before it needs to, undershooting sharp turns. Start
// around 1.5-2.5m and adjust based on your car's actual speed and
// turning radius -- faster cars / wider turning radius want a bigger
// lookahead.
// NOTE: this is a CEILING, not a fixed value -- see ADAPTIVE LOOKAHEAD
// below, which shrinks it automatically as the car nears a waypoint.

const double MIN_LOOKAHEAD_METERS = 0.3;
// Floor for the adaptive lookahead so it never collapses to exactly
// zero right as the car reaches a waypoint.

const int STUCK_UPDATE_LIMIT = 30;
// FAILSAFE trigger for the "circling forever" case. If the distance to the
// current waypoint hasn't meaningfully improved in this many consecutive
// GPS-fix updates, the car is orbiting the point instead of actually
// approaching it (a known limitation of pursuit-style steering when a
// waypoint ends up inside the vehicle's turning radius). At 5Hz fixes, 10
// updates is ~2 seconds -- down from an earlier 40 (~8s), which let the
// car sit there circling for far too long before anything happened.
// Lower this further for an even faster reaction; raise it if legitimate
// slow progress (e.g. a wide, deliberate turn) is being mistaken for
// being stuck.

const double STUCK_PROGRESS_THRESHOLD_METERS = 0.05;
// Minimum improvement in distance-to-waypoint required to reset the stuck
// counter. Smaller than this counts as "no real progress" for that update.

const double STEERING_SLEW_DEG_PER_SEC = 120.0;
// How fast the servo is allowed to travel toward the commanded angle, in
// degrees of servo travel per SECOND of wall time.
//
// This used to be one degree per call to updateSteering(), which was only
// reached on a real GPS fix -- so the ramp rate was silently tied to the
// fix rate. A 25-degree correction needed 25 fixes: 2.5 s at a perfect
// 10 Hz and considerably worse whenever fixes were dropped, by which
// point the car had driven metres past the point where the correction
// was called for. At 120 deg/s the same 25-degree swing takes ~0.2 s and
// takes exactly as long regardless of what GPS is doing.
//
// Raise for a snappier response; lower if the servo slams hard enough to
// unsettle the car. The full 60-degree lock-to-lock sweep takes
// 60 / this many seconds -- keep it within what the servo can physically
// slew, or the commanded angle just runs ahead of the horn.

const unsigned long MAX_STEERING_TICK_MS = 200;
// Ceiling on the time delta a single slew step may integrate. Without it,
// a long stall (GPS dropout, blocking setup call) would be followed by
// one step large enough to snap the servo straight to the target.

// --- DEAD RECKONING (fills the gap between real GPS fixes) ---
const unsigned long DEAD_RECKON_INTERVAL_MS = 100;
// How often to run a predicted-position steering update when no new real
// GPS fix has arrived yet. At 5-10Hz GPS + I2C/NMEA-parsing overhead on
// top, the car could otherwise go 150-300ms+ between steering updates --
// plenty of distance at real driving speed to be well off course before
// anything reacts. This ticks steering forward using dead reckoning in
// between, so corrections start acting on a current estimate instead of
// stalling on stale data. Lower for more frequent (but noisier, since
// extrapolation error grows a little each tick) updates.

const double MAX_DEAD_RECKON_SECONDS = 0.5;
// Safety ceiling on how far ahead predictAhead() is allowed to extrapolate
// in one shot. If GPS drops out for longer than this (lost lock, module
// hiccup), trust the extrapolation less the longer it's been -- past this
// point, skip the prediction tick and just hold the last known position
// rather than compounding a stale heading/speed into a wilder guess.

unsigned long lastFixMillis = 0;
// Timestamp (millis()) of the last real GPS fix OR dead-reckoning tick --
// whichever most recently updated currLocation. Reset at the start of
// each drive so a long setup()/calibration delay doesn't register as a
// huge "time since last fix" on the very first loop() iteration.

const unsigned long STEERING_SETTLE_MS = 800;
// For this long right after handing off from the open-loop maneuver to
// GPS pure-pursuit, cap how far steering is allowed to swing from center,
// regardless of what the raw heading error calls for. currentHeading is
// still converging (each GPS course reading only blends in at
// HEADING_BLEND_ALPHA=0.25 -- see coordinates.cpp) and the very first
// couple of readings right after a standing start are the least
// trustworthy ones GPS produces. This buys a brief straight(ish) stretch
// for the heading estimate to settle on real motion before a big,
// possibly-premature correction is allowed to run -- directly answering
// "drive forward a little before turning hard." Set to 0 to disable.

const int STEERING_SETTLE_MAX_DEVIATION_DEG = 15;
// How far from STR_CENTER steering may swing during the settle window
// above. Kept modest (not full lock) but nonzero, so genuine early
// correction still happens gently -- this isn't "drive dead straight no
// matter what," just "don't let an early bad reading send it to full lock."

unsigned long handoffMillis = 0;
// millis() timestamp of the handoff moment itself, used to measure
// elapsed time against STEERING_SETTLE_MS above.

const double INITIAL_MANEUVER_MAX_METERS = 8.0;
// Safety ceiling for the OPEN-LOOP STARTUP MANEUVER, which now covers two
// legs (start -> WP1 -> WP2) on a single fixed steering angle with no
// live correction at all. This is a hard outer bound only -- in the
// normal case, the GPS arrival check ends the maneuver sooner, as soon as
// it's confirmed close to WP2. Set this a bit above the actual known
// straight-line distance to WP2 (computed in setup() from the fixed
// KNOWN_START/waypoint coordinates) so a slightly-off angle doesn't just
// drive forever, but isn't so tight it cuts the maneuver off short on a
// good run either. Re-check this if your waypoint spacing changes a lot.
// OPEN-LOOP STARTUP MANEUVER. The car's starting position (waypoint 0)
// and starting heading (KNOWN_STARTING_COMPASS_BEARING) are both known
// facts, not GPS measurements -- so the turn needed to face waypoint 1
// can be calculated directly at boot, with no live GPS involved at all.
// This caps how far that maneuver is allowed to drive blind before
// handing off to the normal GPS pure-pursuit loop: just enough to get
// pointed and moving in roughly the right direction. GPS-based
// correction (which already has adaptive lookahead + stuck/recovery
// handling) picks up from there and can act as the backup if this
// maneuver doesn't land the car close enough, or if it stalls/doesn't
// make the progress this maneuver assumed it would.

const double INITIAL_MANEUVER_ARRIVAL_METERS = 1.6;
// Matches the main loop's own arrival radius. Used only during the
// open-loop startup maneuver below, to let GPS confirm early arrival at
// waypoint 1 and cut the blind drive short instead of always running the
// full time-based duration.

const int INITIAL_MANEUVER_THROTTLE = MIN_THROTTLE + 20;
// Deliberately modest and FIXED (not driven by updateCruiseControl's
// speed feedback) -- there's no live speed reading to feed back on yet
// at the very start of this maneuver, since we're intentionally not
// acting on GPS data during it.

// --- STUCK RECOVERY (what happens once STUCK_UPDATE_LIMIT is hit) ---
// Pure pursuit steering can only ever aim the car forward -- it has no
// way to escape a waypoint that has ended up inside the car's physical
// turning radius, which is exactly the situation that trips the stuck
// detector. Previously, hitting that limit just force-declared the
// waypoint "reached" and moved on -- the car never actually got there,
// it just silently gave up. This replaces that with an actual recovery
// maneuver: stop, back up while steering away from the direction that
// got it stuck (the same move a human driver makes to get unstuck), then
// hand back control to normal pursuit steering with a fresh chance to
// converge. Only after MAX_RECOVERY_ATTEMPTS failed attempts at the same
// waypoint does it actually give up -- and it says so explicitly in the
// log, rather than logging a false "reached."
const int REVERSE_THROTTLE = 1400;
// STARTING GUESS, NOT VERIFIED -- needs a value clearly below
// NEUTRAL_THROTTLE (1500) to engage reverse once the double-tap sequence
// below has unlocked it, but the exact threshold varies by ESC and is set
// with the LED program box. ALSO: the ESC must actually be programmed
// into a reverse-enabled running mode ("Forward, reverse with brakes" or
// "Forward and reverse") -- on the factory-default forward-with-brake-only
// mode, no value sent here will make it reverse at all, tap sequence or
// not. Bench-test this value alone (car on blocks, wheels off the ground)
// before trusting it in a recovery maneuver.

const unsigned long RECOVERY_SETTLE_MS = 200;
// Full-neutral pause before the tap sequence starts and after the
// maneuver ends, so the ESC sees genuine rest on both sides of it.

const unsigned long RECOVERY_TAP_MS = 150;
const unsigned long RECOVERY_TAP_GAP_MS = 150;
// This ESC needs an explicit double-tap through neutral to actually
// engage reverse -- confirmed directly, not just assumed from the
// datasheet. A single reverse-range signal from a stop is read as brake,
// not reverse. Sequence: brief reverse pulse (TAP_MS, doesn't move the
// car -- this is purely the "I mean reverse" signal) -> brief neutral
// (TAP_GAP_MS) -> reverse again, which now actually engages and is held
// for RECOVERY_BACKUP_MS below. If reverse still isn't engaging in
// practice, lengthen TAP_MS/TAP_GAP_MS first before assuming something
// else is wrong.

const unsigned long RECOVERY_BACKUP_MS = 600;
// How long to hold the SECOND (real) reverse pulse. Start small -- this
// only needs to move the car far enough that the waypoint is back outside
// the turning radius / adaptive lookahead, not far. Raise it if one
// recovery attempt isn't enough distance to break the circle; lower it if
// the car overshoots backward past a usable approach angle.

const int MAX_RECOVERY_ATTEMPTS = 3;
// After this many failed recovery attempts AT THE SAME WAYPOINT, actually
// give up on it (and log that plainly) rather than retrying forever.

// Computes the bearing (in this project's internal "math angle" convention
// -- 0 = east, counter-clockwise positive, same as path.getCurrentHeading())
// and straight-line distance between two known lon/lat points, using the
// same local-tangent-plane approximation coordinates.cpp uses internally.
//
// Used ONLY by the open-loop startup maneuver in setup(), so that the
// maneuver's drive distance is computed from FIXED, trusted constants
// (KNOWN_START_LON/LAT and this path's own hardcoded waypoint list)
// instead of the live, GPS-calibrated position -- which is only an average
// of a handful of fixes and carries real residual noise, noise that gets
// amplified badly on a short first leg like this one.
void computeKnownBearingAndDistance(double lon1, double lat1, double lon2, double lat2,
                                     double &bearingMathDegOut, double &distanceMetersOut) {
  const double METERS_PER_DEG_LAT_LOCAL = 111320.0;
  double avgLatRad = ((lat1 + lat2) / 2.0) * (PI / 180.0);
  double eastMeters = (lon2 - lon1) * METERS_PER_DEG_LAT_LOCAL * cos(avgLatRad);
  double northMeters = (lat2 - lat1) * METERS_PER_DEG_LAT_LOCAL;
  bearingMathDegOut = atan2(northMeters, eastMeters) * 180.0 / PI;
  distanceMetersOut = sqrt(eastMeters * eastMeters + northMeters * northMeters);
}

void setup() {

  while (!Serial) { ; }

  Serial.begin(9600);
  // Reverted from 115200 -- that produced garbled output in testing
  // (readable text, then corrupted bytes mid-session), which points to
  // this board/cable/USB-serial chip not reliably handling the higher
  // rate rather than a software problem. Not worth the debugging
  // confusion it introduces; 9600 is the safe, well-supported default.

  Wire.begin(); // A4=SDA, A5=SCL on the Uno

  // The SAM-M10Q defaults to 1 fix/second. Push it to 10Hz so position
  // updates (and therefore steering/distance checks) happen 10x more
  // often -- catching drift while it's still small instead of letting a
  // long gap between fixes turn into one big correction. This is a tiny
  // raw UBX-CFG-RATE command -- no library needed. (Was 5Hz; raised
  // further per real-world testing showing steering needed to react
  // sooner, with smaller corrections, rather than occasionally needing
  // a large one.)
  // Trim to GPS+Galileo BEFORE requesting the rate below -- this is what
  // actually makes 10Hz achievable (see function comment for why).
  disableExtraGnssConstellations();
  delay(100); // give the module a moment to apply the new signal config

  sendUbxSetRate(100); // 100ms between fixes = 10Hz
  delay(100); // give the module a moment to apply the new rate

  // The SAM-M10Q outputs standard NMEA sentences (including GGA and RMC)
  // over I2C by default -- no explicit configuration needed to start
  // reading them with the parser below.

  Serial.println(F("--- System Active: Reading GPS over I2C ---"));

  // GATE: don't touch calibration, heading, or the waypoint list until
  // the fix has actually settled. getCurrentGPS() updates satellitesInUse
  // / hdop as a side effect any time it parses a GGA sentence, whether or
  // not that particular call also completes a full RMC sentence -- so it's
  // safe (and necessary) to just keep calling it here and check the two
  // fields directly, rather than waiting on its return value.
  Serial.println(F("Waiting for GPS lock..."));
  unsigned long lockWaitStart = millis();
  while (satellitesInUse < MIN_SATELLITES_FOR_LOCK || hdop > MAX_HDOP_FOR_LOCK) {
    getCurrentGPS();
    if (millis() - lockWaitStart > 2000) { // heartbeat so it's obvious this isn't hung
      Serial.print(F("  still waiting - Sats: ")); Serial.print(satellitesInUse);
      Serial.print(F(" HDOP: ")); Serial.println(hdop);
      lockWaitStart = millis();
    }
  }
  Serial.print(F("GPS lock acquired - Sats: ")); Serial.print(satellitesInUse);
  Serial.print(F(" HDOP: ")); Serial.println(hdop);

  // CALIBRATION: average several consecutive fixes to smooth out random
  // noise, then compare that average against the known true starting
  // coordinates. The difference becomes a correction offset applied to
  // every reading for the rest of this session -- this cancels out
  // whatever systematic bias this particular power-up/reacquisition
  // happened to converge on, which is what was causing whole sessions to
  // come out shifted relative to each other.
  const int CALIBRATION_FIX_COUNT = 10;

  Serial.println(F("Calibrating against known starting location..."));

  // Averaged as offsets FROM the known start rather than as absolute
  // coordinates: the deltas are a few hundred E7 units, so the sum can't
  // overflow an int32 the way ten absolute longitudes would.
  int32_t sumDLonE7 = 0, sumDLatE7 = 0;
  int collected = 0;

  while (collected < CALIBRATION_FIX_COUNT) {
    if (getCurrentGPS()) {
      sumDLonE7 += (xValE7 - KNOWN_START_LON_E7);
      sumDLatE7 += (yValE7 - KNOWN_START_LAT_E7);
      collected++;
    }
  }

  calibOffsetLonE7 = -(sumDLonE7 / CALIBRATION_FIX_COUNT);
  calibOffsetLatE7 = -(sumDLatE7 / CALIBRATION_FIX_COUNT);

  Serial.print(F("Calibration offset - Lon: ")); printDegreesE7(calibOffsetLonE7);
  Serial.print(F(" Lat: ")); printDegreesE7(calibOffsetLatE7); Serial.println();

  // Apply the offset retroactively to the averaged reading so the very
  // first stored location is already corrected.
  xValE7 += calibOffsetLonE7;
  yValE7 += calibOffsetLatE7;
  path.setCurrentLocation(xValE7, yValE7);

  // This car always starts facing roughly the same direction (~144 deg
  // from north, SE). Seeding the heading here means steering has a real
  // reference from the very first loop, instead of driving blind/straight
  // until enough movement builds up a heading estimate on its own.
  const double KNOWN_STARTING_COMPASS_BEARING = 144.0;
  path.setInitialHeading(KNOWN_STARTING_COMPASS_BEARING);

  // inserting all the coordinates

  startXE7 = path.getCurrentLocation().x;

  startYE7 = path.getCurrentLocation().y;

  path.add_point(startXE7, startYE7);

  path.add_point(-1050167711L, 399708088L);

  path.add_point(-1050167990L, 399707661L);

  path.add_point(-1050168352L, 399707112L);

  path.add_point(-1050167993L, 399706823L);

  path.add_point(-1050167172L, 399706728L);

  path.add_point(-1050166487L, 399706992L);

  path.add_point(-1050166014L, 399707620L);

  path.add_point(-1050166020L, 399708347L);

  path.calculate_distances(); // must run after all points are added

  myservo.attach(servoPin);

  motorDriver.attach(escPin, 1000, 2000);

  // ESC ARMING SEQUENCE: Must send neutral signal on startup

  motorDriver.writeMicroseconds(driveValue); // 1500 = Neutral/Stop

  delay(3000); // Wait 3 seconds for the ESC to arm and beep



}

void loop() {
  // GPS-driven line following starts at waypoint 1.
  // The line for each leg is always: previous waypoint -> target waypoint.
  // Steering uses ONLY signed GPS cross-track error. GPS heading/bearing is
  // not used by this controller.
  for (int i = 1; i < path.numPoints(); i++) {
    bool destinationReached = false;
    int activeSteerTarget = STR_CENTER;
    double bestDistSeen = 1e9;
    int stuckCounter = 0;
    int recoveryAttempts = 0;
    lastFixMillis = millis();

    // At the start of a new leg, cross-track error is exactly zero at the
    // waypoint. Give the car a short geometry-derived nudge toward the new
    // leg so the line follower can actually begin the corner.
    if (i > 1) {
      performCornerTransition(i);
    }

    while (!destinationReached) {
      bool gotRealFix = getCurrentGPS();
      unsigned long nowMillis = millis();

      if (!gotRealFix) {
        if (nowMillis - lastFixMillis > GPS_STALE_TIMEOUT_MS) {
          stopDrive();
          continue;
        }
        // Between fixes the last commanded angle is still the best
        // available, and the servo may not have reached it yet -- keep
        // ramping toward it instead of freezing mid-correction until the
        // next fix lands.
        updateSteering(activeSteerTarget);
        continue;
      }

      lastFixMillis = nowMillis;

      // Restore the live coordinate telemetry. This was present in the
      // original project and is important for verifying what the GPS
      // is actually feeding the controller.
      Coordinates liveLocation = path.getCurrentLocation();
      Coordinates targetLocation = path.getCurrentPath(i);

      double currentDistMeters = path.calculateCurrDistance(i);
      double crossTrackErrorMeters = path.crossTrackErrorMeters(i);
      double headingErrorDeg = path.headingErrorToLegDegrees(i);
      int targetSteerAngle = computeTargetSteeringAngle(crossTrackErrorMeters, headingErrorDeg);

      Serial.print(F("WP ")); Serial.print(i + 1);
      Serial.print(F(" | Current: ")); printDegreesE7(liveLocation.x);
      Serial.print(F(", ")); printDegreesE7(liveLocation.y);
      Serial.print(F(" | Target: ")); printDegreesE7(targetLocation.x);
      Serial.print(F(", ")); printDegreesE7(targetLocation.y);
      Serial.print(F(" | Dist: ")); Serial.print(currentDistMeters, 2);
      Serial.print(F(" m | CrossTrack: ")); Serial.print(crossTrackErrorMeters, 3);
      Serial.print(F(" m | HdgErr: ")); Serial.print(headingErrorDeg, 1);
      Serial.print(F(" deg | Steer: ")); Serial.print(targetSteerAngle);
      Serial.print(F(" | Speed: ")); Serial.print(currentSpeedMs, 2);
      Serial.print(F(" m/s | Sats: ")); Serial.print(satellitesInUse);
      Serial.print(F(" | HDOP: ")); Serial.println(hdop, 1);

      // Waypoint arrival radius: ~1.6 m, matching the existing project.
      if (currentDistMeters <= 1.6) {
        Serial.print(F("--- Reached Waypoint "));
        Serial.println(i + 1);
        stopDrive();
        destinationReached = true;
        continue;
      }

      // Safety: if GPS says the car is very far from the active line, do not
      // keep driving toward the next waypoint. Stop rather than driving blind
      // toward a wall, pond, ledge, etc. A reboot/new navigation run is then
      // required after physically putting the car back near the course.
      if (fabs(crossTrackErrorMeters) > MAX_ALLOWED_CROSS_TRACK_ERROR_METERS) {
        Serial.print(F("*** OFF PATH: "));
        Serial.print(fabs(crossTrackErrorMeters), 2);
        Serial.println(F(" m -> STOP ***"));
        stopDrive();
        continue;
      }

      // Existing stuck detector, retained as a secondary failsafe.
      if (currentDistMeters < bestDistSeen - STUCK_PROGRESS_THRESHOLD_METERS) {
        bestDistSeen = currentDistMeters;
        stuckCounter = 0;
      } else {
        stuckCounter++;
      }

      if (stuckCounter >= STUCK_UPDATE_LIMIT) {
        recoveryAttempts++;

        if (recoveryAttempts > MAX_RECOVERY_ATTEMPTS) {
          Serial.print(F("--- WP ")); Serial.print(i + 1);
          Serial.println(F(": giving up after repeated recovery attempts (NOT reached)"));
          stopDrive();
          destinationReached = true;
          continue;
        }

        Serial.print(F("--- WP ")); Serial.print(i + 1);
        Serial.print(F(": stuck - recovery attempt "));
        Serial.println(recoveryAttempts);

        // Same sign convention as crossTrackErrorMeters:
        // negative = car right of line -> recovery steers left; positive -> right.
        performRecoveryManeuver(crossTrackErrorMeters);
        bestDistSeen = 1e9;
        stuckCounter = 0;
        continue;
      }

      int activeMaxThrottle = (i == 1) ? FIRST_GPS_WAYPOINT_MAX_THROTTLE : MAX_THROTTLE;
      activeSteerTarget = targetSteerAngle;
      updateSteering(targetSteerAngle);
      updateCruiseControl(activeMaxThrottle);
    }
  }

  Serial.println(F("--- ALL WAYPOINTS COMPLETED ---"));
  while (true) {
    stopDrive();
    delay(1000);
  }
}

// Combines the two GPS geometry errors for the active leg into one servo
// command.
//
// Geometry conventions (both produced by coordinates.cpp):
//   crossTrackErrorMeters > 0 = car is LEFT of the line  -> steer RIGHT
//   headingErrorDeg       > 0 = leg runs LEFT of the car -> steer LEFT
//
// Both are converted to a single signed "how many degrees of RIGHT do we
// want" number, which STR_RIGHT_SIGN then turns into a servo value. A
// left-of-line error wants right; a leg that runs left of the car's nose
// wants left.
int computeTargetSteeringAngle(double crossTrackErrorMeters, double headingErrorDeg) {
  double crossCorrection = 0.0;
  if (fabs(crossTrackErrorMeters) >= CROSS_TRACK_DEADBAND_METERS) {
    crossCorrection = crossTrackErrorMeters * CROSS_TRACK_KP;
    if (crossCorrection > MAX_CROSS_TRACK_CORRECTION_DEG) crossCorrection = MAX_CROSS_TRACK_CORRECTION_DEG;
    if (crossCorrection < -MAX_CROSS_TRACK_CORRECTION_DEG) crossCorrection = -MAX_CROSS_TRACK_CORRECTION_DEG;
  }

  double headingCorrection = 0.0;
  if (fabs(headingErrorDeg) >= HEADING_DEADBAND_DEG) {
    headingCorrection = headingErrorDeg * HEADING_KP;
    if (headingCorrection > MAX_HEADING_CORRECTION_DEG) headingCorrection = MAX_HEADING_CORRECTION_DEG;
    if (headingCorrection < -MAX_HEADING_CORRECTION_DEG) headingCorrection = -MAX_HEADING_CORRECTION_DEG;
  }

  double rightwardDeg = crossCorrection - headingCorrection;
  int target = STR_CENTER + STR_RIGHT_SIGN * (int)rightwardDeg;
  return constrain(target, STR_MIN, STR_MAX);
}

// Starts the next waypoint leg without using GPS heading. The car begins at
// the waypoint, so cross-track error on the new segment is zero and cannot
// tell a pure line follower which way to turn. We therefore derive the turn
// direction from the incoming and outgoing waypoint vectors, apply a small
// bounded steering bias, and keep it only until the GPS position has moved
// about one meter down the new segment. Normal cross-track control then
// takes over.
void performCornerTransition(int targetIndex) {
  if (targetIndex <= 1 || targetIndex >= path.numPoints()) {
    return;
  }

  Coordinates p0 = path.getCurrentPath(targetIndex - 2);
  Coordinates p1 = path.getCurrentPath(targetIndex - 1);
  Coordinates p2 = path.getCurrentPath(targetIndex);

  const double METERS_PER_E7_DEG = 111320.0e-7;
  double meanLat = ((double)p0.y + (double)p1.y + (double)p2.y) / 3.0 * 1.0e-7;
  double lonScale = METERS_PER_E7_DEG * cos(meanLat * acos(-1.0) / 180.0);

  double inE = (double)(p1.x - p0.x) * lonScale;
  double inN = (double)(p1.y - p0.y) * METERS_PER_E7_DEG;
  double outE = (double)(p2.x - p1.x) * lonScale;
  double outN = (double)(p2.y - p1.y) * METERS_PER_E7_DEG;

  double inLen = sqrt(inE * inE + inN * inN);
  double outLen = sqrt(outE * outE + outN * outN);
  if (inLen < 0.001 || outLen < 0.001) {
    return;
  }

  // Positive cross = geometrically LEFT turn; negative = RIGHT turn.
  double turnCross = inE * outN - inN * outE;
  double turnDot = inE * outE + inN * outN;
  double turnAngleDeg = atan2(turnCross, turnDot) * 180.0 / acos(-1.0);

  if (fabs(turnAngleDeg) < 3.0) {
    return; // effectively straight
  }

  double correction = fabs(turnAngleDeg) * CORNER_TURN_GAIN;
  if (correction > MAX_CORNER_INIT_CORRECTION_DEG) {
    correction = MAX_CORNER_INIT_CORRECTION_DEG;
  }

  // turnAngleDeg > 0 means a LEFT corner; < 0 means a RIGHT corner.
  double cornerRightwardDeg = (turnAngleDeg > 0.0) ? -correction : correction;
  int cornerSteer = STR_CENTER + STR_RIGHT_SIGN * (int)cornerRightwardDeg;

  // constrain() needs its bounds numerically low-then-high. Reversed,
  // every value below the high bound came back as the high bound and the
  // car took a full-lock turn out of every corner regardless of which way
  // the path actually turned.
  cornerSteer = constrain(cornerSteer, STR_MIN, STR_MAX);

  Serial.print(F("--- Corner transition at WP "));
  Serial.print(targetIndex);
  Serial.print(F(" | Turn: "));
  Serial.print(turnAngleDeg, 1);
  Serial.print(F(" deg | Initial steer: "));
  Serial.println(cornerSteer);

  unsigned long transitionStart = millis();
  lastFixMillis = transitionStart;
  setSteeringImmediate(cornerSteer);

  while (millis() - transitionStart < CORNER_TRANSITION_TIMEOUT_MS) {
    bool gotRealFix = getCurrentGPS();
    unsigned long nowMillis = millis();

    if (!gotRealFix) {
      if (nowMillis - lastFixMillis > GPS_STALE_TIMEOUT_MS) {
        stopDrive();
        Serial.println(F("*** GPS STALE DURING CORNER - STOPPED ***"));
        return;
      }
      updateSteering(cornerSteer);
      continue;
    }

    lastFixMillis = nowMillis;

    double alongMeters = path.alongTrackDistanceMeters(targetIndex);
    double crossMeters = path.crossTrackErrorMeters(targetIndex);

    Serial.print(F("Corner | Along: "));
    Serial.print(alongMeters, 2);
    Serial.print(F(" m | Cross: "));
    Serial.print(crossMeters, 2);
    Serial.print(F(" m | Steer: "));
    Serial.println(cornerSteer);

    if (fabs(crossMeters) > MAX_ALLOWED_CROSS_TRACK_ERROR_METERS) {
      stopDrive();
      Serial.println(F("*** OFF PATH DURING CORNER - STOPPED ***"));
      return;
    }

    if (alongMeters >= CORNER_TRANSITION_DISTANCE_METERS) {
      return;
    }

    // Keep the car at a deliberately slow crawl while initiating the turn.
    updateSteering(cornerSteer);
    updateCruiseControl(CORNER_TRANSITION_MAX_THROTTLE);
  }

  stopDrive();
  Serial.println(F("*** CORNER TRANSITION TIMEOUT - STOPPED ***"));
}

// Moves the servo toward targetAngle at STEERING_SLEW_DEG_PER_SEC,
// measured against millis() rather than against the number of calls, and
// returns immediately. Call it as often as possible: extra calls make the
// motion smoother but never faster, and the achieved rate no longer
// depends on how often GPS happens to produce a fix.
//
// commandedAngle is kept as a double so that calls a few milliseconds
// apart accumulate fractional degrees instead of being rounded away to no
// movement at all.
void updateSteering(int targetAngle) {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastSteeringUpdateMillis;
  lastSteeringUpdateMillis = now;
  if (elapsedMs > MAX_STEERING_TICK_MS) elapsedMs = MAX_STEERING_TICK_MS;

  double maxStep = STEERING_SLEW_DEG_PER_SEC * (double)elapsedMs / 1000.0;
  double delta = (double)targetAngle - commandedAngle;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  commandedAngle += delta;

  int rounded = constrain((int)(commandedAngle + 0.5), STR_MIN, STR_MAX);
  if (rounded != currentAngle) {
    currentAngle = rounded;
    myservo.write(currentAngle);
  }
}

// Jumps the servo straight to an angle, bypassing the slew limit, and
// resyncs the ramp state so the next updateSteering() continues from
// where the servo physically is rather than from a stale angle.
void setSteeringImmediate(int angle) {
  currentAngle = constrain(angle, STR_MIN, STR_MAX);
  commandedAngle = currentAngle;
  lastSteeringUpdateMillis = millis();
  myservo.write(currentAngle);
}

const int MAX_THROTTLE_CHANGE_PER_UPDATE = 5;
// RATE LIMIT on driveValue itself -- fixes an overshoot bug where the car
// consistently ran well past TARGET_SPEED (e.g. requesting ~0.7 m/s but
// actually hitting ~1.5 m/s). The root cause: driveValue += throttleAdjustment
// ACCUMULATES on every single GPS fix (5-10Hz), but the car's real speed
// (and the GPS's own speed reading) can't respond that fast -- there's
// real inertia and reporting lag between commanding more throttle and
// currentSpeedMs actually reflecting it. Without a cap, several updates'
// worth of "still too slow" additions stack up on top of each other
// before feedback ever catches up, so driveValue blows way past what's
// actually needed by the time the real speed arrives -- classic integrator
// windup/overshoot, same failure mode this codebase already fixed for
// steering with MAX_TARGET_ANGLE_CHANGE_PER_UPDATE. This caps how much
// driveValue can move in a single update, so each addition gets a chance
// to actually show up in currentSpeedMs before more is piled on.
// Lower this for gentler/slower-converging acceleration; raise it if the
// car is now too sluggish to reach TARGET_SPEED promptly.

// --- PROPORTIONAL CRUISE CONTROL ---
void updateCruiseControl(int activeMaxThrottle) {
  // Calculate the difference between how fast we want to go, and how fast we are going
  double error = TARGET_SPEED - currentSpeedMs;

  // Multiply the error by a tuning factor (Kp). 
  // If we are 1 m/s too slow, add 20 to the throttle.
  int throttleAdjustment = error * 20.0; 

  // Cap the adjustment BEFORE applying it, not after -- limiting driveValue
  // itself after the fact wouldn't stop it from being slammed straight to
  // MAX_THROTTLE in one update whenever the error is large (e.g. right at
  // the start, from a dead stop).
  if (throttleAdjustment > MAX_THROTTLE_CHANGE_PER_UPDATE) throttleAdjustment = MAX_THROTTLE_CHANGE_PER_UPDATE;
  if (throttleAdjustment < -MAX_THROTTLE_CHANGE_PER_UPDATE) throttleAdjustment = -MAX_THROTTLE_CHANGE_PER_UPDATE;

  driveValue += throttleAdjustment;

  // SAFETY LIMITS: Prevent the ESC from commanding dangerous speeds
  if (driveValue > activeMaxThrottle) driveValue = activeMaxThrottle; 
  if (driveValue < MIN_THROTTLE) driveValue = MIN_THROTTLE; 

  motorDriver.writeMicroseconds(driveValue);
}

void stopDrive(){
  driveValue = NEUTRAL_THROTTLE;
  motorDriver.writeMicroseconds(driveValue);
  setSteeringImmediate(STR_CENTER);
}

void performRecoveryManeuver(double steerErrorAtStuck) {
  stopDrive();
  delay(RECOVERY_SETTLE_MS);

  // Same geometry convention as normal steering: positive means the car
  // is LEFT of the line and therefore needs a RIGHT recovery.
  bool wantLeft = (steerErrorAtStuck < 0.0);
  int recoverySteer = wantLeft ? STR_LEFT : STR_RIGHT;
  setSteeringImmediate(recoverySteer);

  // DOUBLE-TAP: confirmed on the bench (not just assumed from the
  // datasheet) that this ESC reads a single reverse-range signal from a
  // full stop as brake, not reverse. A brief reverse pulse, a return to
  // neutral, THEN reverse again is what actually engages reverse -- the
  // first pulse alone never moves the car, it just "arms" the second one.
  motorDriver.writeMicroseconds(REVERSE_THROTTLE);
  delay(RECOVERY_TAP_MS);
  motorDriver.writeMicroseconds(NEUTRAL_THROTTLE);
  delay(RECOVERY_TAP_GAP_MS);

  motorDriver.writeMicroseconds(REVERSE_THROTTLE);
  delay(RECOVERY_BACKUP_MS);

  stopDrive(); // also recentres the servo and resyncs the ramp state
  delay(RECOVERY_SETTLE_MS);
}

// --- Raw I2C access to the u-blox module's DDC interface ---
// Register 0xFD/0xFE (read as a 2-byte big-endian value) reports how many
// bytes are waiting in the module's output buffer. Register 0xFF streams
// those bytes out sequentially on repeated reads. This is the whole
// interface -- no library needed for it.

// Disables GLONASS, BeiDou, and QZSS, leaving GPS + Galileo active, via
// UBX-CFG-VALSET (RAM layer only -- resets on power cycle, nothing
// written to flash). This is what actually unlocks the 10Hz rate
// requested by sendUbxSetRate() below: per the SAM-M10Q datasheet, the
// module's max nav rate DEPENDS on how many constellations it's tracking
// concurrently -- all four (the factory default) caps it at 5Hz
// regardless of what rate you ask for; GPS+Galileo alone supports the
// full 10Hz. Fewer constellations does mean slightly fewer visible
// satellites, but GPS+Galileo together is still plenty for a fix at
// short range like this.
void disableExtraGnssConstellations() {
  const uint32_t GLO_ENA_KEY  = 0x10310025;
  const uint32_t BDS_ENA_KEY  = 0x10310022;
  const uint32_t QZSS_ENA_KEY = 0x10310024;

  uint8_t payload[19];
  payload[0] = 0x00; // message version
  payload[1] = 0x01; // layer bitmask: RAM only (bit0)
  payload[2] = 0x00; payload[3] = 0x00; // reserved

  uint32_t keys[3] = { GLO_ENA_KEY, BDS_ENA_KEY, QZSS_ENA_KEY };
  int idx = 4;
  for (int k = 0; k < 3; k++) {
    payload[idx++] = keys[k] & 0xFF;
    payload[idx++] = (keys[k] >> 8) & 0xFF;
    payload[idx++] = (keys[k] >> 16) & 0xFF;
    payload[idx++] = (keys[k] >> 24) & 0xFF;
    payload[idx++] = 0x00; // value: disabled (these are 1-byte boolean keys)
  }

  uint8_t msgClass = 0x06; // CFG
  uint8_t msgId = 0x8A;    // VALSET
  uint8_t lenLSB = sizeof(payload), lenMSB = 0;

  uint8_t ckA = 0, ckB = 0;
  ckA += msgClass; ckB += ckA;
  ckA += msgId;    ckB += ckA;
  ckA += lenLSB;   ckB += ckA;
  ckA += lenMSB;   ckB += ckA;
  for (uint8_t i = 0; i < sizeof(payload); i++) { ckA += payload[i]; ckB += ckA; }

  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(0xB5); Wire.write(0x62); // UBX sync chars
  Wire.write(msgClass); Wire.write(msgId);
  Wire.write(lenLSB); Wire.write(lenMSB);
  for (uint8_t i = 0; i < sizeof(payload); i++) Wire.write(payload[i]);
  Wire.write(ckA); Wire.write(ckB);
  Wire.endTransmission();
}

// Sends a raw UBX-CFG-RATE command to change how often the module
// produces a fix. measRateMs is milliseconds between fixes (e.g. 200 =
// 5Hz, 100 = 10Hz). This is the whole config command, built by hand --
// no library needed, just a documented ~14-byte binary packet with a
// standard 8-bit Fletcher checksum.
void sendUbxSetRate(uint16_t measRateMs) {
  uint8_t payload[6];
  payload[0] = measRateMs & 0xFF;
  payload[1] = (measRateMs >> 8) & 0xFF;
  payload[2] = 1; payload[3] = 0; // navRate: 1 measurement per nav solution
  payload[4] = 1; payload[5] = 0; // timeRef: 1 = GPS time

  uint8_t msgClass = 0x06; // CFG
  uint8_t msgId = 0x08;    // RATE
  uint8_t lenLSB = 6, lenMSB = 0;

  uint8_t ckA = 0, ckB = 0;
  uint8_t toChecksum[10] = { msgClass, msgId, lenLSB, lenMSB,
                             payload[0], payload[1], payload[2],
                             payload[3], payload[4], payload[5] };
  for (int i = 0; i < 10; i++) {
    ckA += toChecksum[i];
    ckB += ckA;
  }

  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(0xB5); Wire.write(0x62); // UBX sync chars
  Wire.write(msgClass); Wire.write(msgId);
  Wire.write(lenLSB); Wire.write(lenMSB);
  for (int i = 0; i < 6; i++) Wire.write(payload[i]);
  Wire.write(ckA); Wire.write(ckB);
  Wire.endTransmission();
}

uint16_t i2cBytesAvailable() {
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(0xFD);
  if (Wire.endTransmission(false) != 0) return 0; // repeated start, module didn't ack
  Wire.requestFrom((int)GPS_I2C_ADDR, 2);
  if (Wire.available() < 2) return 0;
  uint16_t highByte = Wire.read();
  uint16_t lowByte = Wire.read();
  return (highByte << 8) | lowByte;
}

// Requests up to 32 bytes (the AVR Wire library's internal buffer limit)
// in a SINGLE I2C transaction and leaves them sitting in Wire's own
// buffer. Once called, plain Wire.read()/Wire.available() calls drain
// that buffer with zero further I2C bus activity -- this replaces doing
// a full separate transaction for every individual byte, which was by
// far the bigger bottleneck compared to the module's own output rate.
void i2cRequestChunk(uint16_t maxBytes) {
  uint8_t chunkSize = (maxBytes > 32) ? 32 : (uint8_t)maxBytes;
  Wire.beginTransmission(GPS_I2C_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission(false);
  Wire.requestFrom((int)GPS_I2C_ADDR, (int)chunkSize);
}

// Verifies the NMEA checksum (the *XX hex value at the end of the sentence).
// Rejects sentences that were corrupted in transit.
bool validateChecksum(const char *sentence) {
  if (sentence[0] != '$') return false;

  const char *star = strchr(sentence, '*');
  if (star == NULL) return false;
  if (strlen(star) < 3) return false; // need '*' plus 2 hex digits

  byte checksum = 0;
  for (const char *p = sentence + 1; p < star; p++) {
    checksum ^= (byte)*p;
  }

  char hex[3] = { star[1], star[2], '\0' };
  byte expected = (byte)strtol(hex, NULL, 16);

  return checksum == expected;
}

// Splits a NMEA sentence in place (replaces commas with '\0') and fills
// fieldPtrs[] with pointers to the start of each field. Returns field count.
int splitFields(char *sentence, char *fieldPtrs[], int maxFields) {
  int count = 0;
  char *p = sentence;
  fieldPtrs[count++] = p;
  while (*p != '\0' && count < maxFields) {
    if (*p == ',') {
      *p = '\0';
      fieldPtrs[count++] = p + 1;
    }
    p++;
  }
  return count;
}

// Parses $GPGGA/$GNGGA sentences for satellite count and HDOP (fix quality).
// GGA fields: 0:header 1:time 2:lat 3:NS 4:lon 5:EW 6:fixQuality 7:numSat 8:HDOP
void checkGGA(char *sentence) {
  if (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0) return;
  if (!validateChecksum(sentence)) return;

  char *fields[9];
  int n = splitFields(sentence, fields, 9);
  if (n < 9) return; // sentence too short/malformed

  if (strlen(fields[7]) > 0) satellitesInUse = atoi(fields[7]);
  if (strlen(fields[8]) > 0) hdop = atof(fields[8]);
}

// Converts a raw NMEA DDDMM.MMMMM field (degrees + decimal minutes) into
// 1e-7 degrees, entirely in integer arithmetic. Going through atof()/a
// float here was the original precision loss: the parsed value had
// already been rounded onto a ~0.65 m grid before any navigation math ran.
int32_t nmeaFieldToE7(const char *field) {
  uint32_t wholeMinutes = 0;   // DDDMM as an integer
  uint32_t fracMinutesE5 = 0;  // digits after the decimal point, scaled to 1e-5 minutes
  uint32_t fracScale = 10000UL;
  bool seenDot = false;

  for (const char *p = field; *p != '\0'; p++) {
    if (*p == '.') {
      seenDot = true;
      continue;
    }
    if (*p < '0' || *p > '9') {
      return 0;
    }
    uint8_t digit = (uint8_t)(*p - '0');
    if (!seenDot) {
      wholeMinutes = wholeMinutes * 10UL + digit;
    } else if (fracScale > 0UL) {
      fracMinutesE5 += digit * fracScale;
      fracScale /= 10UL;
    }
  }

  uint32_t degrees = wholeMinutes / 100UL;
  uint32_t minutesE5 = (wholeMinutes % 100UL) * 100000UL + fracMinutesE5;

  // minutes -> degrees is a divide by 60; in E7 units that is
  // minutesE5 * 1e7 / (1e5 * 60) = minutesE5 * 10 / 6. minutesE5 stays
  // below 1e7, so the intermediate fits a uint32.
  return (int32_t)(degrees * 10000000UL + (minutesE5 * 10UL) / 6UL);
}

// Parses $GNRMC/$GPRMC sentences for position, speed, and course-over-ground.
// RMC fields: 0:header 1:time 2:status 3:lat 4:NS 5:lon 6:EW 7:speed 8:course
bool parseRMC(char *sentence) {
  if (strncmp(sentence, "$GNRMC", 6) != 0 && strncmp(sentence, "$GPRMC", 6) != 0) return false;
  if (!validateChecksum(sentence)) return false;

  char *fields[9];
  int n = splitFields(sentence, fields, 9);
  if (n < 8) return false; // need at least through the speed field

  char *latStr   = fields[3];
  char *nsStr    = fields[4];
  char *lonStr   = fields[5];
  char *ewStr    = fields[6];
  char *speedStr = fields[7];

  if (strlen(latStr) == 0 || strlen(lonStr) == 0) return false;

  int32_t parsedYE7 = nmeaFieldToE7(latStr);
  int32_t parsedXE7 = nmeaFieldToE7(lonStr);

  if (nsStr[0] == 'S') parsedYE7 = -parsedYE7;
  if (ewStr[0] == 'W') parsedXE7 = -parsedXE7;

  double parsedSpeed = (strlen(speedStr) > 0) ? atof(speedStr) * 0.514444 : 0.0;

  if (parsedSpeed < 0.0 || parsedSpeed > MAX_PLAUSIBLE_SPEED) return false;

  // Course-over-ground: only present/meaningful if the 9th field exists
  // and isn't blank (many receivers leave it blank at very low speed,
  // since course is meaningless when barely moving).
  double gpsCourseDeg = 0.0;
  bool courseValid = false;
  if (n >= 9 && strlen(fields[8]) > 0) {
    gpsCourseDeg = atof(fields[8]);
    courseValid = true;
  }

  xValE7 = parsedXE7 + calibOffsetLonE7;
  yValE7 = parsedYE7 + calibOffsetLatE7;
  currentSpeedMs = parsedSpeed;

  path.updateFromGPS(xValE7, yValE7, currentSpeedMs, gpsCourseDeg, courseValid);

  return true;
}

// Reads available bytes over I2C and returns true exactly once a full,
// valid RMC sentence (position + speed) has been received. Pulls bytes
// in 32-byte batches via i2cRequestChunk() rather than one I2C
// transaction per byte, then drains each batch cheaply via Wire.read().
bool getCurrentGPS() {
  uint16_t available = i2cBytesAvailable();

  while (available > 0) {
    if (Wire.available() == 0) {
      i2cRequestChunk(available);
      if (Wire.available() == 0) break; // module didn't respond this round
    }

    char inChar = (char)Wire.read();
    available--;

    if (inChar == (char)0xFF) continue; // filler byte, no real data here

    if (inChar == '\n') {
      nmeaBuffer[nmeaIndex] = '\0';
      nmeaIndex = 0;

      checkGGA(nmeaBuffer);
      if (parseRMC(nmeaBuffer)) {
        return true;
      }

    } else if (inChar != '\r') {
      if (inChar >= 32 && inChar <= 126) {
        if (nmeaIndex < NMEA_BUFFER_SIZE - 1) {
          nmeaBuffer[nmeaIndex++] = inChar;
        } else {
          nmeaIndex = 0; // line ran long without a terminator - drop it
        }
      }
    }
  }
  return false;
}