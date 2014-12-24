//
//  wrapper.cpp
//  wrapper
//
//  Created by Connor Monahan on 12/24/14.
//  Copyright (c) 2014 Ratchet Rockers. All rights reserved.
//

#include <iostream>
#include <unistd.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "opencv.hpp"
#include "freenect.hpp"
#include "cvstruct.hpp"

using namespace cv;
using namespace std;

enum InputSource {
	IN_UNK = 0,
	IN_CAMERA,
	IN_KINECT,
	IN_VIDEO,
	IN_PICTURE
};

enum ColorMode {
	C_RGB = 0,
	C_IR
};

VideoCapture * open_kinect(int color, int camera)
{
	VideoCapture * cap;
	if (color == C_IR) {
		ir_init i = {camera};
		cap = new KinectCapture(i);
	} else {
		rgb_init r = {camera};
		cap = new KinectCapture(r);
	}
	return cap;
}

const int BUFLEN = 256;

int check_command(string line)
{
	if (line == "help\n") {
		cout << "wrapper system" << endl;
	} else {
		cout << "unknown '" <<line<<"' type help" << endl;
		return 1;
	}
	return 0;
}

int console()
{
	static fd_set rfds;
	static struct timeval tv;
	static char buf[BUFLEN];
	static int retval;
	
	FD_ZERO(&rfds);
	FD_SET(STDIN_FILENO, &rfds);
	tv.tv_sec = 0;
	tv.tv_usec = 33333;
	retval = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
	if (FD_ISSET(STDIN_FILENO, &rfds)) {
		if (fgets(buf, BUFLEN, stdin)) {
			string line(buf);
			return check_command(line);
		}
	}
	return retval;
}

int loop(VideoCapture * cap)
{
	CvApplication app;
	Mat input, output;
	while (cap->isOpened()) {
		cap->grab();
		cap->retrieve(input, 0);
		if (input.rows < 1 || input.cols < 1)
			throw runtime_error("Camera input image is empty");
		output = app.process(input);
		imshow("Result", output);
		console();
		if ((waitKey(1) & 0xFF) == 27)
			break;
	}
	return 0;
}

struct wrapper_options {
	int camera;
	string file_name;
	enum InputSource input;
	enum ColorMode color;
};

int load_video(struct wrapper_options opt)
{
	VideoCapture *x = NULL;
	switch (opt.input) {
		case IN_KINECT:
			x = open_kinect(opt.color, opt.camera);
			break;
		case IN_CAMERA:
			x = new VideoCapture(opt.camera);
			break;
		case IN_VIDEO:
			if (opt.file_name.empty())
				throw runtime_error("Option -v requires -f argument");
			x = new VideoCapture(opt.file_name);
			break;
		case IN_PICTURE:
			break;
		default:
			throw runtime_error("Unknown input source " + to_string(opt.input));
	}
	loop(x);
	x->release();
	return 0;
}

int main(int argc, char *argv[])
{
	int ch;
	struct wrapper_options opt;
	
	opt.input = IN_CAMERA;
	opt.color = C_RGB;
	opt.camera = 0;
	
	while ((ch = getopt(argc, argv, "C:f:ckvpil")) != -1) {
		switch (ch) {
			case 'C':
				sscanf(optarg, "%d", &opt.camera);
				break;
			case 'f':
				opt.file_name = string(optarg);
				break;
			case 'c':
				opt.input = IN_CAMERA;
				break;
			case 'k':
				opt.input = IN_KINECT;
				break;
			case 'v':
				opt.input = IN_VIDEO;
				break;
			case 'p':
				opt.input = IN_PICTURE;
				break;
			case 'i':
				opt.color = C_IR;
				break;
			case 'l':
				opt.color = C_RGB;
				break;
		}
	}
	try {
		return load_video(opt);
	} catch(exception & ex) {
		cout << "wrapper error: " << ex.what() << endl;
		return 1;
	}
}
