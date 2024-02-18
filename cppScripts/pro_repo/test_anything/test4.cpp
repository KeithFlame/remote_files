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
//    cout << "8位校验和: 0x" << hex << sum << endl;
//
//    cout << "8位校验和: " << data1+Number2HexStr(sum) << endl;
//
//
//    return 0;
//}

int main() {
    // 输入的字符串
    //// 输入的字符串
    //std::string hexString = "24 34 31 30 36 34 31 33";

    //// 使用字符串流分割字符串
    //std::istringstream iss(hexString);
    //std::vector<std::string> hexValues{
    //    std::istream_iterator<std::string>{iss},
    //    std::istream_iterator<std::string>{}
    //};    24 33 31 30 36 34 31 34 

    //// 存储ASCII码对应的字符
    //std::string asciiResult;

    //// 遍历每个16进制字符串
    //for (const auto& hex : hexValues) {
    //    // 将16进制字符串转为10进制
    //    int decimalValue;
    //    std::stringstream converter(hex);
    //    converter >> std::hex >> decimalValue;

    //    // 将10进制数字转为ASCII字符并添加到结果中
    //    asciiResult += static_cast<char>(decimalValue);
    //}

    //// 提取ASCII码的倒数第4位至倒数第3位组合后的字符串
    //std::string asciiSubset = asciiResult.substr(asciiResult.size() - 4, 2);

    //// 将组合后的字符串转为16进制
    //int hexSubset;
    //std::stringstream converter(asciiSubset);
    //converter >> std::hex >> hexSubset;

    //// 输出结果
    //std::cout << "ASCII Result: " << asciiResult << std::endl;
    //std::cout << "Hex Subset: " << hexSubset << std::endl;


    int decimalNumber = 100;

    // 将10进制数字转为16进制字符串
    std::stringstream hexConverter;
    hexConverter << std::hex << decimalNumber;
    std::string hexString = hexConverter.str();

    // 将每位数值对应的ASCII码转为16进制数字
    std::string asciiHexString;
    for (char c : hexString) {
        int asciiValue = static_cast<int>(c);
        std::stringstream asciiHex;
        asciiHex << std::hex << asciiValue;
        std::cout <<"1: " << std::atoi(asciiHex.str().c_str()) << std::endl;
        std::cout <<"2: 0x" << asciiValue << std::endl;
        asciiHexString += asciiHex.str();
    }

    // 输出结果
    std::cout << "Decimal: " << decimalNumber << ", Hex: 0x" << hexString << std::endl;
    std::cout << "ASCII Hex String: " << asciiHexString << std::endl;

    //return 0;
    return 0;
}   