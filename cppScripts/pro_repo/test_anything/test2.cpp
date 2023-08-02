#include <Eigen/Core>
#include <iostream>
#include <string>

std::string addBinary(std::string a, std::string b) {
    if (a == "0")
        return b;
    if (b == "0")
        return a;
    int len_a = a.length(), len_b = b.length();
    std::string aa(a), bb(b), cc;
    int a_b = len_a - len_b;
    if (a_b > 0)
        while (a_b--)
            bb = "0" + bb;
    else
    {
        a_b = -a_b;
        while (a_b--)
            aa = "0" + aa;
    }
    bool jinwei = 0;
    std::cout << "aa: " << aa << std::endl;
    std::cout << "bb: " << bb << std::endl;
    cc.resize(aa.length(), '0');
    for (int i = aa.length() - 1; i >= 0; i--)
    {
        if (a[i] == '0' && b[i] == '1' && jinwei == 1)
        {
            cc[i] = '0';
            jinwei = 1;
        }
        else if (a[i] == '0' && b[i] == '1' && jinwei == 0)
        {
            cc[i] = '1';
            jinwei = 0;
        }
        else if (a[i] == '1' && b[i] == '0' && jinwei == 1)
        {
            cc[i] = '0';
            jinwei = 1;
        }
        else if (a[i] == '1' && b[i] == '0' && jinwei == 0)
        {
            cc[i] = '1';
            jinwei = 0;
        }
        else if (a[i] == '1' && b[i] == '1' && jinwei == 1)
        {
            cc[i] = '1';
            jinwei = 1;
        }
        else if (a[i] == '1' && b[i] == '1' && jinwei == 0)
        {
            cc[i] = '0';
            jinwei = 1;
        }
        else if (a[i] == '0' && b[i] == '0' && jinwei == 1)
        {
            cc[i] = '1';
            jinwei = 0;
        }
        else if (a[i] == '0' && b[i] == '0' && jinwei == 0)
        {
            cc[i] = '0';
            jinwei = 0;
        }

        ;
    }
    if (jinwei)
        cc = "1" + cc;
    return cc;
}

int _1main()
{
    std::string a, b,c;
    a = "100";
    b = "110010";
    c = addBinary(a, b);
    std::cout << "res: " << c << std::endl;

	return 0;

}