#include "moveit2_samples/position_parser.hpp"

Position::Position(float x, float y, float z) : x_{x}, y_{y}, z_{z} {}

PositionParser::PositionParser(std::string filepath) : file_{filepath}{

    if(!file_){
        std::cerr<<"JSON file not found: "<< filepath<<"\n";
        return;
    }

    nlohmann::json j;
    file_ >> j;
        
    for(const auto& item : j){
        float x = item.value("x",0.0f);
        float y = item.value("y",0.0f);
        float z = item.value("z",0.0f);
        position_.emplace_back(x,y,z);
    }

}