#pragma once
#pragma once
#include "tinyxml2.h"
#include <string>
#include <vector>
#include "eigen_extensions.h"
#include "info_define.h"

#pragma execution_character_set("utf-8") //set encoding character

namespace Keith {
	bool readTrocarInfo(std::vector<double>& trocar, std::pair<InfoSymbol, std::string>& str);
	bool readTrocarBaseInfo(std::vector<double>& trocar, std::pair<InfoSymbol, std::string>& str);
	bool readTargetInfo(std::vector<double>& all_type, std::pair<InfoSymbol, std::string>& str);

}