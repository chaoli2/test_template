// Main of the test script

#include <iostream>
#include "test.h"

int main()
{
  std::cout << "begin of main" << std::endl;

  TestExample test_inst(3);
  int index = test_inst.getIndex();
  
  std::cout << "index is " << index << " now" << std::endl; 
  std::cout << "end of main" << std::endl;
}
