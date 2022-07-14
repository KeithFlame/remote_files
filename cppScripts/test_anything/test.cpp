#include <iostream>

int main()
{
	char data[4] = { 1070,41,42 };
	for (size_t i = 0; i < sizeof(data) / sizeof(char); i++)
		std::cout <<"data: " << int(data[i]) << std::endl;
	for (auto i:data)
		std::cout << "data: " << int(i) << std::endl;
	return 0;
}