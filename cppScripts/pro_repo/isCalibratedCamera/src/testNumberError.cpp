// testNumberError.cpp: 定义应用程序的入口点。
//

#include "testNumberError.h"
#include "basicMath.h"


DataProcessing::DataProcessing():compare_result(10.f)
{
    compareProximalandDistalMarkerDev();
}

void DataProcessing::getDataResult()
{
    if (compare_result > 0.4f)
        std::cout << "\n\nERROR::NEED_RECALIBRATE_CAMERA!!!\n\n" << std::endl;
    else
        std::cout << "\n\nPASS::CAMERA_IS_FINE.\n\n" << std::endl;
}

float DataProcessing::calcMaxDev(std::string fpath)
{
	//std::string fpath = "./varifyTrocarCoords_C1.log";
	readLog.open(fpath);
	if (!readLog.is_open())
	{
		std::cout << "ERROR::FILE_OPEN_FAILED!!!"<<std::endl;
		return 10000.f;
	}
	char buf[2000];
    std::vector<std::string>  result;
    std::vector<float>  a,b,c;
    int i = 0;
	while (readLog.getline(buf,sizeof(buf)))
	{
        result = split(buf, " ,*\t");
        if (result.size() < 2)
            continue;
        a.emplace_back(std::atof(result[0].c_str()));
        b.emplace_back(std::atof(result[1].c_str()));
        c.emplace_back(std::atof(result[2].c_str()));
	}
    readLog.close();
    float max_X(0.f), max_Y(0.f), max_Z(0.f);

    max_X = abs(math::getMaxFloat(a) - math::getMinFloat(a));
    max_Y = abs(math::getMaxFloat(b) - math::getMinFloat(b));
    max_Z = abs(math::getMaxFloat(c) - math::getMinFloat(c));
    float res = max_X > max_Y ? (max_X > max_Z ? max_X : max_Z) : (max_Y > max_Z ? max_Y : max_Z);
    return res;
    
}

void DataProcessing::compareProximalandDistalMarkerDev()
{
    float dev_proximal = calcMaxDev("./arm10001/varifyTrocarCoords_C1.log");
    float dev_distal = calcMaxDev("./arm10001/varifyTrocarCoords_C2.log");
    compare_result = dev_proximal > dev_distal ? dev_proximal : dev_distal;
    return ;
}

std::vector<std::string> DataProcessing::split(const std::string& s, const std::string& seperator) {
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while (i != s.size()) {
        //找到字符串中首个不等于分隔符的字母；
        int flag = 0;
        while (i != s.size() && flag == 0) {
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[i] == seperator[x]) {
                    ++i;
                    flag = 0;
                    break;
                }
        }

        //找到又一个分隔符，将两个分隔符之间的字符串取出；
        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0) {
            for (string_size x = 0; x < seperator.size(); ++x)
                if (s[j] == seperator[x]) {
                    flag = 1;
                    break;
                }
            if (flag == 0)
                ++j;
        }
        if (i != j) {
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}