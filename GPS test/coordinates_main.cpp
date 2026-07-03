#include <iostream>
#include "coordinates.h"
#include <fstream>
#include <sstream>
using namespace std;

int main() {
    std::ifstream file("coordinates_test.txt");   
    Path path;

    if (!file) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    string line;
    while (getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str;
        getline(ss, x_str, ',');
        getline(ss, y_str, ',');
        int x = std::stoi(x_str);
        int y = std::stoi(y_str);
        path.add_point(x, y);
    }
    file.close();
    path.calculate_distances();
    path.print_path();


    int xVal, yVal;
    for(int i = 0 ;i< path.numPoints(); i++){
        while(path.calculateCurrDistance(i) > 0.5){
            cout<<endl<<"What is your current x"<<endl;
            cin>>xVal;

            cout<<"What is your current y"<<endl;
            cin>>yVal;

            path.setCurrentLocation(xVal, yVal);
            if(path.calculateCurrDistance(i) > 0.5){
                cout<< path.steering_direction(i, path.getCurrentLocation())<<endl;
                cout<<"Distance to destination "<<i+1<<": "<<path.calculateCurrDistance(i)<<endl;
            }else{
                cout<<"You have reached destination "<<i+1<<endl;
            }
        }
    }
    
    return 0;
}

