// C++ Program to demonstrate
// Use of template
#include <iostream>
#include "cpp_project_examples/template.hpp"

using namespace std;

template <typename T>   
double mult(T x, T y)
{
    return x.y; 
}


int main()
{
    // Call mult for int
    cout << mult<int>(3, 7) << endl;
    // call mult for double
    cout << mult<double>(3.0, 7.0) << endl;

    return 0;
}
