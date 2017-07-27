#include "ros/ros.h"
#include "ugv/motorvels.h"
#include <stdlib.h>
#include <ncurses.h>

#define speed 100

int main(int argc, char** argv) {
	ros::init(argc, argv, "keyboardcontrol");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<ugv::motorvels>("motors",1000);
	ros::Rate loop_rate(1);

	// Prepare ncurses
	initscr();
	clear();
	noecho();
	cbreak();
	WINDOW *menu_win = newwin(800,600,0,0);
	keypad(menu_win, TRUE);

	while(ros::ok()) {
		ugv::motorvels msg;

		int ch = wgetch(menu_win);

		msg.left = 0;
		msg.right = 0;

		switch(ch) {
			case KEY_LEFT: msg.left = -speed; msg.right=speed;break;
			case KEY_RIGHT: msg.left = speed; msg.right=-speed;break;
			case KEY_UP: msg.left = speed; msg.right=speed;break;
			case KEY_DOWN: msg.left = -speed; msg.right=-speed;break;
		};

		pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	endwin();
	return 0;
}

