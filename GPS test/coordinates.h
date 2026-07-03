#ifndef PATH_H
#define PATH_H

#include <vector>

using namespace std;

/**
 * @brief Structure to represent a 2D coordinate point.
 */
struct Coordinates {
    int x;
    int y;
};

/**
 * @brief Structure to represent polar vector parameters (magnitude and direction).
 */
struct Direction {
    double distance; // Magnitude from the origin
    double angle;    // Angle in degrees from the positive X-axis
};

/**
 * @brief Class to manage a path of sequential 2D points and compute geometric transformations.
 */
class Path {
public:
   
    void add_point(int x, int y);

    
    void calculate_distances();

    string steering_direction(int index, Coordinates currPosition);

   void print_path();
   int numPoints();

   void setCurrentLocation(int x, int y);
   
   Coordinates getCurrentLocation();

   double calculateCurrDistance(int index);

private:
    std::vector<Coordinates> points;
    std::vector<Direction> directions;
    Coordinates currLocation;
};

#endif 