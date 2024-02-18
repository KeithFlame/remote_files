////#include <opencv.hpp>
////#include "timer.h"
//#include <vector>
//int main()
//{
//	//cv::Mat frame;
//	//
//
//	//frame = cv::imread("F:\\code_git\\pythonScripts\\open_camera\\pic\\0001.png",0);
//	//auto tic = mmath::timer::getCurrentTimePoint();
//	//cv::medianBlur(frame,frame,9);
//	//float t1 = mmath::timer::getDurationSince(tic);
//	//std::cout << "duration: " << t1 << "\t" << std::endl;
//
//	return 0;
//}

#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>

using namespace std;

std::string Number2HexStr(uint32_t mData) {
    std::stringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)mData;
    std::string hex_str = ss.str();
    if (1 == hex_str.size())
        hex_str = "0" + hex_str;
    return hex_str;
}

//int main() {
//    vector<int> data = { 0x24, 0x32, 0x32, 0x30, 0x32, 0x39 };
//    std::string data1 = "$410";
//    int lv = 0;
//    data1 += Number2HexStr(lv);
//    int sum = 0;
//    for (int i = 0; i < data.size(); i++) {
//        sum ^= data1[i];
//        cout << (int)data1[i] << "    " << data[i] << endl;
//    }
//        
//    //sum = sum & 0xff;
//    cout << "8λУ���: 0x" << hex << sum << endl;
//
//    cout << "8λУ���: " << data1+Number2HexStr(sum) << endl;
//
//
//    return 0;
//}

int main() {
    // ������ַ���
    //// ������ַ���
    //std::string hexString = "24 34 31 30 36 34 31 33";

    //// ʹ���ַ������ָ��ַ���
    //std::istringstream iss(hexString);
    //std::vector<std::string> hexValues{
    //    std::istream_iterator<std::string>{iss},
    //    std::istream_iterator<std::string>{}
    //};    24 33 31 30 36 34 31 34 

    //// �洢ASCII���Ӧ���ַ�
    //std::string asciiResult;

    //// ����ÿ��16�����ַ���
    //for (const auto& hex : hexValues) {
    //    // ��16�����ַ���תΪ10����
    //    int decimalValue;
    //    std::stringstream converter(hex);
    //    converter >> std::hex >> decimalValue;

    //    // ��10��������תΪASCII�ַ�����ӵ������
    //    asciiResult += static_cast<char>(decimalValue);
    //}

    //// ��ȡASCII��ĵ�����4λ��������3λ��Ϻ���ַ���
    //std::string asciiSubset = asciiResult.substr(asciiResult.size() - 4, 2);

    //// ����Ϻ���ַ���תΪ16����
    //int hexSubset;
    //std::stringstream converter(asciiSubset);
    //converter >> std::hex >> hexSubset;

    //// ������
    //std::cout << "ASCII Result: " << asciiResult << std::endl;
    //std::cout << "Hex Subset: " << hexSubset << std::endl;


    int decimalNumber = 100;

    // ��10��������תΪ16�����ַ���
    std::stringstream hexConverter;
    hexConverter << std::hex << decimalNumber;
    std::string hexString = hexConverter.str();

    // ��ÿλ��ֵ��Ӧ��ASCII��תΪ16��������
    std::string asciiHexString;
    for (char c : hexString) {
        int asciiValue = static_cast<int>(c);
        std::stringstream asciiHex;
        asciiHex << std::hex << asciiValue;
        std::cout <<"1: " << std::atoi(asciiHex.str().c_str()) << std::endl;
        std::cout <<"2: 0x" << asciiValue << std::endl;
        asciiHexString += asciiHex.str();
    }

    // ������
    std::cout << "Decimal: " << decimalNumber << ", Hex: 0x" << hexString << std::endl;
    std::cout << "ASCII Hex String: " << asciiHexString << std::endl;

    //return 0;
    return 0;
}   