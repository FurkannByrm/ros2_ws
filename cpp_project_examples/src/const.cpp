#include "cpp_project_examples/const.hpp"

int MyClass::getNumber() const {
    return number;
}

int main(){

    MyClass my_class;
    std::cout<<my_class.getNumber();

    return 0;
}