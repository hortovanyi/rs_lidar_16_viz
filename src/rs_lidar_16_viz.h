#pragma once

enum CameraAngle
{
	XY, TopDown, Side, FPS
};

struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};