#ifndef _TEMPLATE_HPP_
#define _TEMPLATE_HPP_

#include <iostream>

template <typename T> 
class Array{
public:
    Array(T arr[], int s);
    void print();

private:
    T* ptr;
    int size;

};

template <typename T>
Array<T>::Array(T arr[], int s)
{

}



#endif //_TEMPLATE_HPP_