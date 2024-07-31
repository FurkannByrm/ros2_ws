#include <iostream>

bool echo(int& number){
    if(number == 0)
    {
        return false;
    }
    std::cout<<"hello"<<std::endl;
    return true;
}

int main(){
int num;
do
{
    num = 0;
    std::cout<< "(0: exit, 1: echo)=>";
    std::cin>>num;
} while (echo(num));
 


return 0;

}