#include <iostream>
#include <malloc.h>
using namespace std;

int **p;

int main()
{
  p = (int **)malloc(4 * sizeof(int *));
  cout << *p << endl;
  return 0;
}
