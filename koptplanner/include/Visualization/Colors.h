#pragma once

class ColorRGB
{
private:
  const static int s_max_id = 19;
  static ColorRGB s_current_color;
  static int s_current_id;

public:
  double r, g, b;

  ColorRGB(double r, double g, double b);
  ~ColorRGB();
  static int init();
  static void cycle();
  static ColorRGB getCurrent();
};