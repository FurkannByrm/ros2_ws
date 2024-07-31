#include <iostream>

using namespace std;

int main()
{
repeat:
  try
  { // error can occured in scope
    int number1, number2;
    cout << "sign dividing by number: ";
    cin >> number1;
    cout << "sign dividing : ";
    cin >> number2;
    if (number2 == 0){
      throw 101; // catch allow us to go when error occure
    }
    cout << "result : " << number1 / float(number2);
  }

  catch (int errorCode)
  { // error statement
    cout << "dividing not '0' ..." << endl;
    goto repeat;
  }

  return 0;
}