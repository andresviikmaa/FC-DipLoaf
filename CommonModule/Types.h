#pragma once
#define PI 3.14159265
#define TAU (2*PI)
#include <vector>
#include <opencv2/core.hpp>

struct ColorRange
{
	int low;
	int high;
};

struct HSVColorRange
{
	ColorRange hue;
	ColorRange sat;
	ColorRange val;
};

struct Speed
{
	union {
		struct {
			double velocity;
			double heading;
			double rotation;
		};
		struct
		{
			double x;
			double y;
			double r;
		};
	};
};
typedef struct {
	double x;
	double y;
} Point;
typedef struct {
	double a;
	double r;
} PolarPoint;

struct ObjectPosition
{
	bool isValid;
	cv::Point2d fieldCoords = cv::Point2d(INT_MAX, INT_MAX); // (x, y) Coordinates to display objects on field by, relative to field
	cv::Point2i rawPixelCoords; // (x, y) Raw from frame
	cv::Point2d polarMetricCoords;      // (distance, angle) Relative to robot
};

struct BallPosition: public ObjectPosition
{
	int id;
	bool isUpdated;
};
struct GatePosition : public ObjectPosition
{
	cv::Point2d minCornerPolarCoords;
};
struct RobotPosition : public ObjectPosition
{

};

//for conf file
const int ID_COM = 1;
const int ID_REF = 2;

//for actual ID info is sent to
const int ID_MAIN_BOARD = 5;

enum OBJECT
{
	BALL = 0, BLUE_GATE, YELLOW_GATE, FIELD, INNER_BORDER, OUTER_BORDER, NUMBER_OF_OBJECTS, SIGHT_MASK
};


enum GameMode {
	//		GAME_MODE_STOPED = 0,
	GAME_MODE_START_SINGLE_PLAY = 0,

	GAME_MODE_PLACED_BALL,
	GAME_MODE_END_HALF,

	GAME_MODE_START_OUR_KICK_OFF,
	GAME_MODE_START_OPPONENT_KICK_OFF,

	GAME_MODE_START_OUR_THROWIN,
	GAME_MODE_START_OPPONENT_THROWIN,

	GAME_MODE_START_OUR_FREE_KICK,
	GAME_MODE_START_OPPONENT_FREE_KICK,

	GAME_MODE_START_OUR_INDIRECT_FREE_KICK,
	GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK,

	GAME_MODE_START_OUR_GOAL_KICK,
	GAME_MODE_START_OPPONENT_GOAL_KICK,

	GAME_MODE_START_OUR_CORNER_KICK,
	GAME_MODE_START_OPPONENT_CORNER_KICK,

	GAME_MODE_START_OUR_PENALTY,
	GAME_MODE_START_OPPONENT_PENALTY,

	GAME_MODE_START_OUR_GOAL,
	GAME_MODE_START_OPPONENT_GOAL,

	GAME_MODE_START_OUR_YELLOW_CARD,
	GAME_MODE_START_OPPONENT_YELLOW_CARD,

	/* our states */
	CAME_MODE_CATCH_KICK_OFF,
	GAME_MODE_IN_PROGRESS,
	GAME_MODE_TAKE_BALL, // other robot passed pall
};
enum RobotColor {
	ROBOT_COLOR_YELLOW_UP = 0,
	ROBOT_COLOR_BLUE_UP
};

