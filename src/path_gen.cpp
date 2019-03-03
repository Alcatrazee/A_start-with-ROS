#include <iostream>
#include <vector>
#include <stdlib.h>
#include <ctime>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
using namespace std;

#define Height 100
#define Width 100

#define array_size 10000

#define obstacle_mark 100

#define heuristic_scaler 0

#define cost 10

#define row 0
#define column 1
#define fn 2
#define parent_row 3
#define parent_col 4
#define valid_bit 5
#define gn 6
#define valid 1
#define invalid 0

#define length 1000

#define hypotenous 1
#define normal 0

#define x_bias -50
#define y_bias -50

// To switch output cost map every cycle
//#define debug_output

/*
	error code  0: no sloution to the map
	error code -1:
	error code -2: start point is coincident to target
*/

void Display_map(int map[Height][Width]);
void Create_map(int map[Height][Width], int cost_map[Height][Width], char show_map[Height][Width]);
int A_star_path_finding(int map[Height][Width], int cost_map[Height][Width], int start_point[2], int end_point[2], int Path[length][fn]);
int Get_heuristic_function(int current_point[2], int Goal[2]);
int check_end_point_in_openset(int end_point[2], int OpenSet[array_size][7]);
int Get_minimun_fn_coord(int Openset[array_size][7]);
int Get_gn(int current_point[3], char dir, int end_point[2]);
void Expend_current_point(int map[Height][Height], int cost_map[Height][Height], int current_point[3], int Openset[array_size][7], int ClosedSet[array_size][7], int end_point[2], int start_point[2], int closed_list_counter);
bool Is_openset_empty(int Openset[array_size][7]);
void Init(int Openset[array_size][7], int ClosedSet[array_size][7]);
bool Is_in_open_or_closed_set(int point[2], int Openset[array_size][7], int Closedset[array_size][7], int closed_list_counter);
int Find_index(int set[array_size][7], int point[2]);
int Find_parent_index(int parent[2], int ClosedSet[array_size][7], int CloseSet_index);
int Get_path(int ClosedSet[array_size][7], int CloseSet_index, int Path[length][fn], int start_point[2]);
void Draw_map_with_path(char show_map[Height][Width], int Path[length][fn], int Path_length);
char Check_start_end_point(int map[Height][Width], int start_point[2], int end_point[2]);

// create a map using 2-D array
void Create_map(int map[Height][Width], int cost_map[Height][Width], char show_map[Height][Width])
{
	for (int i = 0; i < Height; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			if ((i >= 3 && i <= 12 && j >= 12 && j <= 15) || ((i >= 12 && i <= 15 && j >= 3 && j <= 12) || (i == 12 && j >= 15 && j <= 19)))
			{
				cost_map[i][j] = map[i][j] = obstacle_mark;
				show_map[i][j] = 'x';
			}
			else
			{
				cost_map[i][j] = map[i][j] = 0;
				show_map[i][j] = '0';
			}
		}
	}
}

// path finding function,can be alternate to other algorithm like Dijstra
int A_star_path_finding(int map[Height][Width], int cost_map[Height][Width], int start_point[2], int end_point[2], int Path[length][fn])
{
	// Set {row_coordinate,column_coodrinate,f(n),x_parent,y_parent,valid_bit}
	int ClosedSet[array_size][7];
	int OpenSet[array_size][7];
	int current_point[3]; // row col gn
	int min_index = 0;
	int CloseSet_index = 0;
	int Path_length = 0;

	if (start_point[0] == end_point[0] && start_point[1] == end_point[1])
		return -2;
	Init(OpenSet, ClosedSet);

	OpenSet[0][row] = start_point[row];
	OpenSet[0][column] = start_point[column];
	OpenSet[0][fn] = Get_heuristic_function(start_point, end_point);
	OpenSet[0][parent_row] = start_point[parent_row];
	OpenSet[0][parent_col] = start_point[parent_col];
	OpenSet[0][valid_bit] = valid;
	OpenSet[0][gn] = 0;
	cost_map[start_point[row]][start_point[column]] = OpenSet[0][fn];

	// get cost_map
	while (check_end_point_in_openset(end_point, OpenSet) == -1)
	{

		if (!Is_openset_empty(OpenSet))
		{
			min_index = Get_minimun_fn_coord(OpenSet);
			current_point[row] = OpenSet[min_index][row];
			current_point[column] = OpenSet[min_index][column];
			current_point[2] = OpenSet[min_index][gn];
			Expend_current_point(map, cost_map, current_point, OpenSet, ClosedSet, end_point, start_point, CloseSet_index);

			ClosedSet[CloseSet_index][row] = OpenSet[min_index][row];
			ClosedSet[CloseSet_index][column] = OpenSet[min_index][column];
			ClosedSet[CloseSet_index][fn] = OpenSet[min_index][fn];
			ClosedSet[CloseSet_index][parent_row] = OpenSet[min_index][parent_row];
			ClosedSet[CloseSet_index][parent_col] = OpenSet[min_index][parent_col];
			ClosedSet[CloseSet_index][valid_bit] = valid;

			OpenSet[min_index][valid_bit] = invalid;
			CloseSet_index++;

#ifdef debug_output
			Display_map(cost_map);
			cout << endl;
#endif
		}
		else
		{
			return 0;
		}
	}
	int index = check_end_point_in_openset(end_point, OpenSet);
	ClosedSet[CloseSet_index][row] = OpenSet[index][row];
	ClosedSet[CloseSet_index][column] = OpenSet[index][column];
	ClosedSet[CloseSet_index][fn] = OpenSet[index][fn];
	ClosedSet[CloseSet_index][parent_row] = OpenSet[index][parent_row];
	ClosedSet[CloseSet_index][parent_col] = OpenSet[index][parent_col];
	ClosedSet[CloseSet_index][valid_bit] = valid;
	OpenSet[index][valid_bit] = invalid;
	CloseSet_index++;
	Path[0][row] = end_point[row];
	Path[0][column] = end_point[column];
	Path_length = Get_path(ClosedSet, CloseSet_index, Path, start_point);
	return Path_length;
}

// Get path
int Get_path(int ClosedSet[array_size][7], int CloseSet_index, int Path[length][fn], int start_point[2])
{
	int Path_index = 1;
	int parent[2];
	int parent_index = 0;
	parent[row] = ClosedSet[CloseSet_index - 1][parent_row];
	parent[column] = ClosedSet[CloseSet_index - 1][parent_col];
	//	cout << "Path:" << endl;
	//	cout << "(" << Path[0][row] << "," << Path[0][column] << ")" << endl;
	while (!(Path[Path_index - 1][row] == start_point[row] && Path[Path_index - 1][column] == start_point[column]))
	{
		parent_index = Find_parent_index(parent, ClosedSet, CloseSet_index);
		Path[Path_index][row] = ClosedSet[parent_index][row];
		Path[Path_index][column] = ClosedSet[parent_index][column];
		parent[row] = ClosedSet[parent_index][parent_row];
		parent[column] = ClosedSet[parent_index][parent_col];
		//		cout << "(" << Path[Path_index][row] << "," << Path[Path_index][column] << ")" << endl;
		Path_index++;
	}
	return Path_index;
}

// Find parent coordinate index from closed set
int Find_parent_index(int parent[2], int ClosedSet[array_size][7], int CloseSet_index)
{
	for (int i = CloseSet_index - 1; i >= 0; i--)
	{
		if (parent[row] == ClosedSet[i][row] && parent[column] == ClosedSet[i][column])
		{
			return i;
		}
	}
	return 0;
}

//initialzation of close set and open set,just set valid_bit to 0
void Init(int Openset[array_size][7], int ClosedSet[array_size][7])
{
	for (int i = 0; i < array_size; i++)
	{
		Openset[i][valid_bit] = ClosedSet[i][valid_bit] = 0;
		Openset[i][fn] = ClosedSet[i][fn] = 0;
		Openset[i][gn] = ClosedSet[i][gn] = 0;
	}
}

// check if the point is in the open set or closed set.
bool Is_in_open_or_closed_set(int point[2], int Openset[array_size][7], int Closedset[array_size][7], int closed_list_counter)
{
	for (int i = 0; i < array_size; i++)
	{
		if (Openset[i][valid_bit] == 1 && (Openset[i][row] == point[0] && Openset[i][column] == point[1]))
		{
			return true;
		}
	}
	for (int i = 0; i < closed_list_counter; i++)
	{
		if (Closedset[i][row] == point[0] && Closedset[i][column] == point[1])
		{
			return true;
		}
	}
	return false;
}

// check if openset is empty or not
bool Is_openset_empty(int Openset[array_size][7])
{
	for (int i = 0; i < array_size; i++)
	{
		if (Openset[i][valid_bit] == 1)
			return false;
	}
	return true;
}

void Expend_current_point(int map[Height][Height], int cost_map[Height][Height], int current_point[3], int Openset[array_size][7], int ClosedSet[array_size][7], int end_point[2], int start_point[2], int closed_list_counter)
{

	int point[2];

	// upper
	point[0] = current_point[0] - 1;
	point[1] = current_point[1];
	if (current_point[0] > 0 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, normal, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}
	// upper left
	point[0] = current_point[0] - 1;
	point[1] = current_point[1] - 1;
	if (current_point[1] > 0 && current_point[0] > 0 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, hypotenous, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}

	// upper right
	point[0] = current_point[0] - 1;
	point[1] = current_point[1] + 1;
	if (current_point[1] < Width - 1 && current_point[0] > 0 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, hypotenous, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}

	// left
	point[0] = current_point[0];
	point[1] = current_point[1] - 1;
	if (current_point[1] > 0 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, normal, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}
	// right
	point[0] = current_point[0];
	point[1] = current_point[1] + 1;
	if (current_point[1] < Width - 1 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{

		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, normal, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}

		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}
	// down
	point[0] = current_point[0] + 1;
	point[1] = current_point[1];
	if (current_point[0] < Height - 1 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, normal, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}

	// lower left
	point[0] = current_point[0] + 1;
	point[1] = current_point[1] - 1;
	if (current_point[1] > 0 && current_point[0] < Height - 1 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, hypotenous, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}

	// lower right
	point[0] = current_point[0] + 1;
	point[1] = current_point[1] + 1;
	if (current_point[1] < Width - 1 && current_point[0] < Height - 1 && !Is_in_open_or_closed_set(point, Openset, ClosedSet, closed_list_counter) && map[point[0]][point[1]] != obstacle_mark)
	{
		// create a point variable
		int index = 0;
		cost_map[point[0]][point[1]] = Get_heuristic_function(point, end_point) + Get_gn(current_point, hypotenous, end_point);
		for (int i = 0; i < array_size; i++)
		{
			if (Openset[i][valid_bit] == 0)
			{
				index = i;
				break;
			}
			else if (i == array_size)
			{
				cout << "open list fulled." << endl;
			}
		}
		Openset[index][row] = point[0];
		Openset[index][column] = point[1];
		Openset[index][fn] = cost_map[point[0]][point[1]];
		Openset[index][parent_row] = current_point[0];
		Openset[index][parent_col] = current_point[1];
		Openset[index][valid_bit] = 1;
		Openset[index][gn] = Get_gn(current_point, normal, end_point);
	}
}

// get Manhaton distance
int Get_heuristic_function(int current_point[2], int Goal[2])
{
	//return sqrt((current_point[0] - Goal[0])*(current_point[0] - Goal[0]) + (current_point[1] - Goal[1])*(current_point[1] - Goal[1]))*cost*heuristic_scaler;
	return (abs(Goal[1] - current_point[1]) + abs(Goal[0] - current_point[0])) * cost * heuristic_scaler;
}

int Get_gn(int current_point[3], char dir, int end_point[2])
{
	static const int cost_hypo = sqrt(cost * cost + cost * cost);
	int gn_ = 0;
	switch (dir)
	{
	case normal:
		gn_ = current_point[2] + cost;
		break;
	case hypotenous:
		gn_ = current_point[2] + cost_hypo;
		break;
	}
	return gn_;
}

int check_end_point_in_openset(int end_point[2], int OpenSet[array_size][7])
{
	for (int i = 0; i < array_size; i++)
	{
		if (OpenSet[i][row] == end_point[0] && OpenSet[i][column] == end_point[1])
			return i;
	}
	return -1;
}

int Get_minimun_fn_coord(int Openset[array_size][7])
{
	int result_index = 0;
	int minimun = 0;
	int i = 0;
	for (i = 0; i < array_size; i++)
	{
		if (Openset[i][valid_bit] == 1)
		{
			minimun = Openset[i][fn];
			result_index = i;
			break;
		}
	}
	for (; i < array_size; i++)
	{
		if (Openset[i][fn] < minimun && Openset[i][valid_bit] == 1)
		{
			result_index = i;
			minimun = Openset[i][fn];
		}
	}
	return result_index;
}

// display the map with grid
void Display_map(int map[Height][Width])
{
	for (int i = 0; i < Height; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			printf("%3d", map[i][j]);
		}
		cout << endl;
	}
}

void Draw_map_with_path(char show_map[Height][Width], int Path[length][fn], int Path_length)
{
	for (int i = 0; i < Path_length; i++)
	{
		show_map[Path[i][row]][Path[i][column]] = '#';
	}
	for (int i = 0; i < Height; i++)
	{
		for (int j = 0; j < Width; j++)
		{
			printf("%2c", show_map[i][j]);
		}
		cout << endl;
	}
}

// Here to check if the start point and end point are coincident with any obstacle
char Check_start_end_point(int map[Height][Width], int start_point[2], int end_point[2])
{
	char code = 0;
	if (map[start_point[0]][start_point[1]] == obstacle_mark)
		code |= 0x01;
	if (map[end_point[0]][end_point[1]] == obstacle_mark)
		code |= 0x10;
	return code;
}

nav_msgs::Path path;
void map_rec_callback(const nav_msgs::OccupancyGrid map)
{
	int Path[length][fn];					// to store a path, can be changed to some variables array with no length declaration
	clock_t start_time, end_time; // to calculate how long the algorithm takes
	int start_point[2] = {0, 0}, end_point[2] = {92, 92};
	int Path_length;

	ROS_INFO("map got!\r\n");
	const uint32_t map_height = 100;
	const uint32_t map_width = 100;
	int map_arr[map_height][map_width], cost_map[map_height][map_width];
	char show_map[map_height][map_width];
	int count = 0;
	for (int i = 0; i < map.info.height; i++)
	{
		for (int j = 0; j < map.info.width; j++)
		{
			cost_map[i][j] = map_arr[i][j] = map.data[count];
			if (map.data[count] == obstacle_mark)
				show_map[i][j] = 'x';
			else
				show_map[i][j] = '.';
			count++;
		}
	}
	cout << map_height << " " << map_width << endl;
	//Create_map(map, cost_map, show_map); // create maps with grid cell , 0 represents navigatable terrain and 'x' or -1 represents the no-go zone
	switch (Check_start_end_point(map_arr, start_point, end_point))
	{
	case 0x01:
		cout << "Error:start point is coincident with obstacle." << endl;
		break;
	case 0x10:
		cout << "Error:end point is coincident with obstacle." << endl;
		break;
	case 0x11:
		cout << "Error:two point are coincident with obstacles." << endl;
		break;
	default:
		cout << "original map" << endl; //
		Display_map(map_arr);
		cout << endl;

		start_time = clock();																																// start the clock ticking
		Path_length = A_star_path_finding(map_arr, cost_map, start_point, end_point, Path); // core function
		end_time = clock();																																	// stop ticking clock

		switch (Path_length)
		{
		case 0:
			cout << "no solution to the map" << endl;
			break;
		case -2:
			cout << "start point is coincident to the goal." << endl;
			break;
		default:
			cout << "total runtime:" << (double)(end_time - start_time) / CLOCKS_PER_SEC << endl;
			cout << "There is ";
			cout << Path_length << " steps in total" << endl;
			Draw_map_with_path(show_map, Path, Path_length);
			/* cout << endl
					 << "cost map" << endl;
			Display_map(cost_map); */

			geometry_msgs::PoseStamped poses[Path_length];
			path.header.frame_id = "map";
			path.header.stamp = ros::Time::now();
			for (int i = 0; i < Path_length; i++)
			{
				poses[i].header.frame_id = "map";
				poses[i].header.stamp = ros::Time::now();
				poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);
				poses[i].pose.position.x = Path[i][1] + x_bias;
				poses[i].pose.position.y = Path[i][0] + y_bias;
				poses[i].pose.position.z = 0;
				path.poses.push_back(poses[i]);
				cout << "(" << poses[i].pose.position.x << "," << poses[i].pose.position.y << ")" << endl;
			}
		}
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "path_finding_node");
	ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
	ros::Subscriber suber = nh.subscribe("/map", 1, map_rec_callback);
	ros::Rate rate(1);

	while (ros::ok())
	{
		ros::spinOnce();
		path_pub.publish(path);
		//path.poses.clear();
		rate.sleep();
	}

	return 0;
}
