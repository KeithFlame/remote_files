// testNumberError.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。

#pragma once
#ifndef _TEST_NUMBER_ERROR_h_
#define _TEST_NUMBER_ERROR_h_
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

class DataProcessing
{
public:
	DataProcessing();
	void getDataResult();
private:
	float calcMaxDev(std::string fpath);
	void compareProximalandDistalMarkerDev();
	std::vector<std::string> split(const std::string& s, const std::string& seperator);
	std::fstream readLog;
	float compare_result;
};


#endif // _TEST_NUMBER_ERROR_h_


// TODO: 在此处引用程序需要的其他标头。
