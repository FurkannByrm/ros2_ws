#include <iostream>
#include <vector>

int main()
{

    std::vector<int> v1 = {11};
    std::vector primes{1,1,2,3,4};// hold the first 5 prime numbers (as int)
    std::vector<int> data(22);
    std::cout<<"an int is : "<<sizeof(int)<< "bytes\n";
    std::cout<<&(primes[0])<<std::endl;
    std::cout<<&(primes[1])<<std::endl;
    std::cout<<&(primes[2])<<std::endl;

    std::cout<<data[1]<<std::endl;
    std::cout<<data[3]<<std::endl;
    std::cout<<data[4]<<std::endl;
    std::cout<<data[5]<<std::endl;
    std::cout<<data.at(1)<<std::endl;

    std::cout<<"data vector size : "<< data.size()<<std::endl;
    std::cout<<"Capacity : "<<data.capacity()<<"Length : "<<data.size()<<std::endl;
    

    return 0;

}