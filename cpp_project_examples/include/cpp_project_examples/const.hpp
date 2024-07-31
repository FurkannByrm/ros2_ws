#ifndef _CONST_HPP_
#define _CONST_HPP_
#include <rclcpp/rclcpp.hpp>
#include <iostream>
// #define INITIATION number{123}, height{2.3}

class MyClass {
    public: 
    MyClass() : number{123}, height{2.3} {}
    // MyClass() : INITIATION {}
    int getNumber()const;
    private:
    const int number;
    const double height;
};



#endif //_CONST_HPP_
