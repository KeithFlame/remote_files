#include "auxiliary_function.h"
#include <direct.h>
#include <io.h>
#include <iomanip> // ��������С���㾫��
#include <sstream> // �����ַ�����

#pragma execution_character_set("utf-8")

int auxiliary_func::isExists(const std::string file_path) {
	if (_access(file_path.c_str(), 0) == -1) { //�жϸ��ļ����Ƿ����
		int flag = _mkdir(file_path.c_str());  //Windows�����ļ���
		if (flag == 0) {  //�����ɹ�
			return 0;
		}
		else { //����ʧ��
			return -1;
		}
	}
	else {
		return 1;
	}
}

std::string auxiliary_func::removeAngleBrackets(const std::string& input) {
	std::string result;
	bool insideAngleBrackets = false;

	for (char c : input) {
		if (c == '<') {
			insideAngleBrackets = true;
		}
		else if (c == '>') {
			insideAngleBrackets = false;
		}
		else if (!insideAngleBrackets) {
			result += c;
		}
	}

	return result;
}

std::string auxiliary_func::fromVec2String(const Eigen::Vector7d& input)
{
	std::string str = "";
	for (size_t i = 0; i < (size_t)input.size(); i++) {
		std::stringstream stream;
		stream << std::fixed << std::setprecision(3) << input[i];
		std::string formattedDouble = stream.str();
		str = str + "  " + formattedDouble;
	}
	return str;
}
