#include <cstdio>

class Base
{
public:
    ~Base() { ::printf("base\n"); }
};

class Derived
    : public Base
{
public:
    ~Derived() { ::printf("derived\n"); }
};

int main(){
    Base *ptr = new Derived;
    delete ptr;
}

