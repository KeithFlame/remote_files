#include "conf_read.h"


//#pragma execution_character_set("utf-8") //set encoding character


bool Keith::readTrocarInfo(std::vector<double>& trocar, std::pair<InfoSymbol, std::string>& str) {
	tinyxml2::XMLDocument conf_read;									// ÅäÖÃÎÄ¼þÔÄ¶Á¾ä±ú
	trocar.clear();
	int i = 0;
	std::string tool_list = "./conf/public/Trocar.xml";
	conf_read.LoadFile(tool_list.c_str());
	if (conf_read.Error()) {
		str.first = InfoSymbol::INFO_ERROR;
		str.second = "FAILURE::Cannot open " + tool_list + ".";
		trocar.resize(12);
		return false;
	}

	tinyxml2::XMLElement* root = conf_read.RootElement();
	root = root->FirstChildElement();
	while (root != NULL) {
		trocar.emplace_back(std::atof(root->GetText()));
		root = root->NextSiblingElement();
		i++;
	}
	str.first = InfoSymbol::INFO_SUCCESS;
	str.second = "SUCCESS::Load Trocar Info.";
	return true;
}

bool Keith::readTrocarBaseInfo(std::vector<double>& trocar, std::pair<InfoSymbol, std::string>& str) {
	tinyxml2::XMLDocument conf_read;									// ÅäÖÃÎÄ¼þÔÄ¶Á¾ä±ú
	trocar.clear();
	int i = 0;
	std::string tool_list = "./conf/public/TrocarBase.xml";
	conf_read.LoadFile(tool_list.c_str());
	if (conf_read.Error()) {
		str.first = InfoSymbol::INFO_ERROR;
		str.second = "FAILURE::Cannot open " + tool_list + ".";
		trocar.resize(12);
		return false;
	}

	tinyxml2::XMLElement* root = conf_read.RootElement();
	root = root->FirstChildElement();
	while (root != NULL) {
		trocar.emplace_back(std::atof(root->GetText()));
		root = root->NextSiblingElement();
		i++;
	}
	str.first = InfoSymbol::INFO_SUCCESS;
	str.second = "SUCCESS::Load Trocar Coords Info.";
	return true;
}

bool Keith::readTargetInfo(std::vector<double>& all_type, std::pair<InfoSymbol, std::string>& str) {
	tinyxml2::XMLDocument conf_read;									// ÅäÖÃÎÄ¼þÔÄ¶Á¾ä±ú
	all_type.clear();
	int i = 0;
	std::string tool_list = "./conf/public/Target.xml";
	conf_read.LoadFile(tool_list.c_str());
	if (conf_read.Error()) {
		str.first = InfoSymbol::INFO_ERROR;
		str.second = "FAILURE::Cannot open " + tool_list + ".";
		all_type.resize(6);
		return false;
	}

	tinyxml2::XMLElement* root = conf_read.RootElement();
	root = root->FirstChildElement();
	while (root != NULL) {
		all_type.emplace_back(std::atof(root->GetText()));
		root = root->NextSiblingElement();
		i++;
	}
	str.first = InfoSymbol::INFO_SUCCESS;
	str.second = "SUCCESS::Load Target Info.";
	return true;
}
