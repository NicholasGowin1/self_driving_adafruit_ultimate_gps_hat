#include "coordinates.h"
#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

void Path::add_point(int x, int y) {
    points.push_back({x, y});
}

void Path::calculate_distances() {
    const double PI = std::acos(-1.0);
    directions.push_back({0, 0});
    for (size_t i = 1; i < points.size(); ++i) {
        double distance = std::sqrt(pow((points[i].x - points[i-1].x), 2) + pow((points[i].y - points[i-1].y), 2));
        double angle = std::atan2(points[i].y - points[i-1].y, points[i].x - points[i-1].x) * 180.0 / PI;
        directions.push_back({distance, angle});
    }
}

string Path::steering_direction(int index, Coordinates currPosition) {
    const double PI = std::acos(-1.0);
    if (index < 0 || index >= points.size()){
        return "invalid index";
    }
    double angle = atan2((points[index].y - currPosition.y), (points[index].x - currPosition.x)) * 180.0 / PI;
    if(angle < 0){
        angle += directions[index].angle;
    }else{
        angle -= directions[index].angle;
    }
    if(angle < 0){
        return "right";
    }else if(angle > 0){
        return "left";
    }else{
        return "straight";
    }

}

void Path::print_path() {
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "Point " << i << ": (" << points[i].x << ", " << points[i].y << ") -> "
                  << "Dist: " << directions[i].distance << ", Angle: " << directions[i].angle << "°\n";
    }
}

int Path::numPoints() {
    return points.size();
}

void Path::setCurrentLocation(int x, int y) {
    currLocation.x = x;
    currLocation.y = y;
}

Coordinates Path::getCurrentLocation(){
    return currLocation;
}


double Path::calculateCurrDistance(int index) {
    if (index < 0 || index >= points.size()) {
        return -1;  // Invalid index
    }
    double dx = points[index].x - currLocation.x;
    double dy = points[index].y - currLocation.y;
    return std::sqrt(dx * dx + dy * dy);
}


