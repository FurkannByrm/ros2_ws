#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include  <deque>


struct Position{
    // Position() = default;
    Position(float x, float y, float z );
    float x_;
    float y_; 
    float z_;
    
    
};



class PositionParser{

    public:
    PositionParser(std::string filepath);

    protected:
    std::deque<Position> position_;
    
    private:
    std::fstream file_;
};