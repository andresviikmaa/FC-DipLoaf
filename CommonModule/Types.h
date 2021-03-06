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
	struct {
		double velocity;
		double heading;
		double rotation;
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
	bool isPredicted = 1;

	double distance;
	double angle;
	double heading;
	cv::Point2d fieldCoords = cv::Point2d(INT_MAX, INT_MAX); // (x, y) Coordinates to display objects on field by, relative to field
	cv::Point2d rawPixelCoords; // (x, y) Raw from frame
	cv::Point2d polarMetricCoords;      // (distance, angle) Relative to robot
};

struct BallPosition: public ObjectPosition
{
	uchar id;
	bool isUpdated;
	union {
		double lostTime;
		double speed; // for simulator
	};
};
struct GatePosition : public ObjectPosition
{
	cv::Point2d minCornerPolarCoords = cv::Point2d(INT_MAX, INT_MAX);
};
enum RobotPos: uchar {
	ROBOT_POS_UNKNOWN = 0,
	ROBOT_POS_GATES_FOUND,
	ROBOT_POS_FIXED
};
struct RobotPosition : public ObjectPosition
{
	short wheelSpeeds[4];
	RobotPos fix = (RobotPos)99;
};

//for conf file
const int ID_COM = 1;
const int ID_REF = 2;

//for actual ID info is sent to
const int ID_MAIN_BOARD = 5;

enum OBJECT: uchar
{
	BLUE_GATE = 0, YELLOW_GATE, BALL, FIELD, INNER_BORDER, OUTER_BORDER, TEAM_PINK, TEAM_PURPLE, NUMBER_OF_OBJECTS, SIGHT_MASK
};


enum GameMode1vs1 : uchar {
	//		
	GAME_MODE_START_PLAY = 0,
	GAME_MODE_STOPED
};

enum GameMode2vs2 : uchar {
	GAME_MODE_PLACED_BALL = 2,
	GAME_MODE_END_HALF,

	GAME_MODE_START_OUR_KICK_OFF,
	GAME_MODE_START_OPPONENT_KICK_OFF,

	GAME_MODE_START_OUR_FREE_KICK,
	GAME_MODE_START_OPPONENT_FREE_KICK,

	GAME_MODE_START_OUR_INDIRECT_FREE_KICK,
	GAME_MODE_START_OPPONENT_INDIRECT_FREE_KICK,

	GAME_MODE_START_OUR_PENALTY,
	GAME_MODE_START_OPPONENT_PENALTY,

	GAME_MODE_START_OUR_GOAL,
	GAME_MODE_START_OPPONENT_GOAL,

	GAME_MODE_START_OUR_YELLOW_CARD,
	GAME_MODE_START_OPPONENT_YELLOW_CARD,

	/* our states */
	GAME_MODE_CATCH_KICK_OFF,
	GAME_MODE_IN_PROGRESS,
	GAME_MODE_TAKE_BALL, // other robot passed pall
};
enum RobotColor : uchar {
	ROBOT_COLOR_YELLOW_UP = 0,
	ROBOT_COLOR_BLUE_UP
};

