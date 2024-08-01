#include <iostream>


class MyClass {
    public:
    MyClass()
    {
        std::cout<<"myclass default ctor this : "<<this<<std::endl;

    }
    ~MyClass()
    {
        std::cout<<"myclass destructor this : "<< this<<std::endl;
    }
    MyClass(const MyClass &other){
        std::cout<< "myclass copy ctor this : "<<this<<std::endl;
        std::cout<< "&other : "<< &other<<std::endl;
    }

};

int main(){

    MyClass m1; //myclass default ctor this : 0x7ffc6800fbe6
    std::cout<<"&m1 = "<<&m1<<std::endl; //&m1 = 0x7ffc6800fbe6
    MyClass m2 = m1; // callback cctor function for m2 and it is sand m1 as argument.
    // myclass copy ctor this : 0x7ffc6800fbe7
    // &other : 0x7ffc6800fbe6
    std::cout<<"&m2 : "<<&m2<<std::endl;
    // &m2 : 0x7ffc6800fbe7

    // myclass destructor this : 0x7ffc6800fbe7 (for m2)
    // myclass destructor this : 0x7ffc6800fbe6 (for m1)
    
   

    return 0;
}