#include "FieldState.h"
#include <map>

FieldState gFieldState;

//TODO: find better place for this
std::pair<OBJECT, std::string> objects[] = {
	std::pair<OBJECT, std::string>(BALL, "Ball"),
	std::pair<OBJECT, std::string>(BLUE_GATE, "Blue Gate"),
	std::pair<OBJECT, std::string>(YELLOW_GATE, "Yellow Gate"),
	std::pair<OBJECT, std::string>(FIELD, "Field"),
	std::pair<OBJECT, std::string>(INNER_BORDER, "Inner Border"),
	std::pair<OBJECT, std::string>(OUTER_BORDER, "Outer Border"),
	//	std::pair<OBJECT, std::string>(NUMBER_OF_OBJECTS, "") // this is intentionally left out

};

std::map<OBJECT, std::string> OBJECT_LABELS(objects, objects + sizeof(objects) / sizeof(objects[0]));
