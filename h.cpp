#include <iostream>

using namespace std::literals;

void print() {
  std::cout << "Ok" << std::endl;
  std::cout << std::endl;
}

int f();
void g(int a) {
  std::cout << a << std::endl;
}



int main() {
  [[maybe_unused]] int a{};
  int b;
  std::cout << a << std::endl;
  long c{118'237'4837};
  std::cout << b << std::endl;
  std::cout << f() << std::endl;
  std::cout << sizeof(int) << " " << sizeof(long) << " " << sizeof(long long) << std::endl;
  std::cout << sizeof(float) << " " << sizeof(double) << " " << sizeof(long double) << std::endl;
  std::cout << sizeof(unsigned int) << " " << sizeof(unsigned long) << " " << sizeof(unsigned long long) << std::endl;
  std::cout << 10e1 << std::endl;
  g(static_cast<int>(7.8));
  std::cout << std::hex << 71 << std::endl;
  std::cout << "Ok"s << std::endl;
  std::string_view s{"Oks"};
  std::cout << s << std::endl;

}

