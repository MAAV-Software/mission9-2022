#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include "json.hpp"
#include "WGS84toCartesian.hpp"
#include "Eigen/Dense"

using json = nlohmann::json;
using namespace std;

json parseItem(json item) {
    json parsed;
    parsed["Latitude"] = item["params"][4];
    parsed["Longitude"] = item["params"][5];
    parsed["Altitude"] = item["params"][6];
    return parsed;
};

json generateItem(array<double,2> waypoint, json pylon, int sequence) {
    //ignoring altitude - leave as default
    //pylon["Altitude"] = waypoint["Altitude"];
    pylon["doJumpId"] = sequence;
    pylon["params"][4] = waypoint[0];
    pylon["params"][5] = waypoint[1];
   // pylon["params"][6] = waypoint["Altitude"];
    return pylon;
}

Eigen::Vector2d calcLinePoint(Eigen::Vector2d start, Eigen::Vector2d along, double t) {
    Eigen::Vector2d result = along * t + start;
    return result;
}

vector<Eigen::Vector2d> calcVecLine(Eigen::Vector2d start, Eigen::Vector2d along, double length, double increment, double zero = 0.0) {
    vector<Eigen::Vector2d> line; //todo declare on heap
    for (double i = zero + i; i < length; i+= increment) {
        line.push_back(calcLinePoint(start, along, i)); //ignores start point
    }
    line.push_back(calcLinePoint(start, along, length + zero));
    return line;
}

double calcVecDistance(Eigen::Vector2d start, Eigen::Vector2d end) {
    Eigen::Vector2d tmp = start - end;
    return tmp.norm();
}

vector<array<double, 2>> vecToArray(vector<Eigen::Vector2d> coordinates) {
    vector<array<double,2>> converted;
    for(int i = 0; i < coordinates.size(); i++) {
       converted.push_back({coordinates[i].x(), coordinates[i].y()});
    }
    return converted;
}

vector<array<double,2>> cartToWGS84(vector<array<double,2>> cartesian, array<double, 2> reference) {
    vector<array<double,2>> WGS84;
    for(int i = 0; i < cartesian.size(); i++) {
        WGS84.push_back(wgs84::fromCartesian(reference, cartesian[i]));
    }
    return WGS84;
}

vector<array<double, 2>> rectangularPath(json launch, json pylon1, json pylon2, json mast, int margin = 5) {
    //units in meters, lat/long = coordinates
    //can't just treat it like x and y b/c the earth is round!!
    //not sure if cartesian coordinates are also in meters... functions relative to reference
    //todo figure out actual units
    //todo best way to do geographic Coordinate Math
    array<double, 2> roundLaunch = {launch["Latitude"], launch["Longitude"]};
    array<double, 2> cartLaunch = wgs84::toCartesian(roundLaunch, roundLaunch); //reference, convert

    array<double, 2> roundPylon1 = {pylon1["Latitude"], pylon1["Longitude"]};
    array<double, 2> cartPylon1 = wgs84::toCartesian(roundLaunch, roundPylon1); //reference, convert

    array<double, 2> roundPylon2 = {pylon2["Latitude"], pylon2["Longitude"]};
    array<double, 2> cartPylon2 = wgs84::toCartesian(roundLaunch, roundPylon2); //reference, convert

    array<double, 2> roundMast = {mast["Latitude"], mast["Longitude"]};
    array<double, 2> cartMast = wgs84::toCartesian(roundLaunch, roundMast); //reference, convert

    //as a test
    cout << "roundpylon1: " << roundPylon1[0] << roundPylon1[1] << endl;
    cout << "cartpylon1: " << cartPylon1[0] << cartPylon1[1] << endl;
    //should be able to do normal math on cartesian coordinates
    //using vector equations to easily get points along the line
    //can't assume orientation will match N/S/E/W
    Eigen::Vector2d vecLaunch( cartLaunch[0], cartLaunch[1]);
    Eigen::Vector2d vecPylon1(cartPylon1[0], cartPylon1[1]);
    Eigen::Vector2d vecPylon2(cartPylon2[0], cartPylon2[1]);
    Eigen::Vector2d vecLand(cartMast[0], cartMast[1]);

    //key vectors: need unit vectors
    Eigen::Vector2d vecP1P2 = vecPylon2 - vecPylon1;
    vecP1P2 = vecP1P2 / vecP1P2.norm();
    Eigen::Vector2d vecRevP1P2 = vecP1P2 * -1;
    Eigen::Vector2d vecNormP1P2(-vecP1P2.x(), vecP1P2.y());
    Eigen::Vector2d vecRevNormP1P2 = vecNormP1P2 * -1;

    //vectors to points: actual size
    Eigen::Vector2d vecBotLeft = vecPylon1 + vecRevP1P2 + vecRevNormP1P2;
    Eigen::Vector2d vecBotRight = vecPylon2 + vecP1P2 + vecRevNormP1P2;
    Eigen::Vector2d vecTopLeft = vecPylon1 + vecRevP1P2 + vecNormP1P2;
    Eigen::Vector2d vecTopRight = vecPylon2 + vecP1P2 + vecNormP1P2;

    //start and end: unit vectors
    Eigen::Vector2d vecLaunchStart = vecBotLeft - vecLaunch;
    vecLaunchStart = vecLaunchStart / vecLaunchStart.norm();
    Eigen::Vector2d vecEndLand = vecLand - vecTopLeft;
    vecEndLand = vecEndLand / vecEndLand.norm();


    //should generate a coordinate every meter
    int increment = 1; //meters
    vector<Eigen::Vector2d> coordinates;
    vector<Eigen::Vector2d> generated;
    generated = calcVecLine(vecLaunch, vecLaunchStart, calcVecDistance(vecLaunch, vecBotLeft), increment);
    coordinates.insert(coordinates.end(), generated.begin(), generated.end());
    generated = calcVecLine(vecBotLeft, vecP1P2, calcVecDistance(vecBotLeft, vecBotRight), increment);
    coordinates.insert(coordinates.end(), generated.begin(), generated.end());
    generated = calcVecLine(vecBotRight, vecNormP1P2, calcVecDistance(vecBotRight, vecTopRight), increment);
    coordinates.insert(coordinates.end(), generated.begin(), generated.end());
    generated = calcVecLine(vecTopRight, vecRevP1P2, calcVecDistance(vecTopRight, vecTopLeft), increment);
    coordinates.insert(coordinates.end(), generated.begin(), generated.end());
    generated = calcVecLine(vecTopLeft, vecEndLand, calcVecDistance(vecTopLeft, vecLand), increment);
    coordinates.insert(coordinates.end(), generated.begin(), generated.end());

    //convert vectors to arrays
    vector<array<double, 2>> coordinateArr = vecToArray(coordinates);
    //convert back to wgs84 coordinates (system used by gps)
    //todo: find documentation on QGroundControl's coordinate system
    vector<array<double, 2>> wgs84Arr = cartToWGS84(coordinateArr, roundLaunch);
    //convert back to waypoints
    return wgs84Arr;
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cout << "Usage: MAAV_Path_Generator [input].plan [output].plan [type]" << std::endl;
        return 1;
    }
    std::cout << "Warning: May not properly account for curvature of the earth" << endl;
    cout << "Warning: Does not consider vertical distances/optimizations" << endl;
    string inputFile = argv[1];
    string outputFile = argv[2];
    string pathType = argv[3];

    ifstream input;
    input.open(inputFile);
    if (!input.is_open()) {
        cout << "Usage: MAAV_Path_Generator [input].plan [output].plan" << std::endl;
        cout << "Unable to open " << inputFile << endl;
        return 1;
    }

    ofstream output;
    output.open(outputFile);
    if (!output.is_open()) {
        cout << "Usage: MAAV_Path_Generator [input].plan [output].plan" << std::endl;
        cout << "Unable to open " << outputFile << endl;
        return 1;
    }
    //files are ready

    json plan;
    input >> plan;
    plan["groundStation"] = "MAAV_path_generator";

    json launch = parseItem(plan["mission"]["items"][0]); //probably better if using a struct
    json pylon1 = parseItem(plan["mission"]["items"][1]);
    json pylon2 = parseItem(plan["mission"]["items"][2]);
    json mast = parseItem(plan["mission"]["items"][3]);

    //for testing - able to receive data
    cout << "launch: " << launch << endl;
    cout << "pylon1: " << pylon1 << endl;
    cout << "pylon2: " << pylon2 << endl;
    cout << "mast: " << mast << endl;

    if (pathType == "Rectangular") {
        //looks like cartesian coordinates are in meters
        vector<array<double, 2>> waypoints = rectangularPath(launch, pylon1, pylon2, mast);
        json launchOriginal = plan["mission"]["items"][0];
        json mastOriginal = plan["mission"]["items"][3];
        json pylonSample = plan["mission"]["items"][1];
        plan["mission"]["items"] = json::array();
        plan["mission"]["items"].push_back(launchOriginal);
        for (int i = 0; i < waypoints.size(); i++) {
            plan["mission"]["items"].push_back(generateItem(waypoints[i], pylonSample, i+1));
        }
        mastOriginal["doJumpId"] = waypoints.size() + 1;
        plan["mission"]["items"].push_back(mastOriginal);
    } else if (pathType == "Circular") {
        cout << "Circular paths are currently unsupported" << endl;
        return 1;
    } else if (pathType == "Spline") {
        cout << "Spline paths are currently unsupported" << endl;
        //using space curves?
        return 1;
    } else {
        cout << "[type] must be: Rectangular, Circular, or Spline" << endl;
        return 1;
    };

    //std::cout << "Hello, World!" << std::endl;

    //save data:
    output << plan;

    //cleanup
    input.close();
    output.close();
    return 0;
}

