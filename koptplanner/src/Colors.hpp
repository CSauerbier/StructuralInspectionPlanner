//TO-DO: Improve code quality

#include "Visualization/Colors.h"

ColorRGB ColorRGB::s_current_color(0,0,0);
int ColorRGB::s_current_id;


ColorRGB::ColorRGB(double r, double g, double b)
{
  this->r = r;
  this->g = g;
  this->b = b;
}

ColorRGB::~ColorRGB()
{
}

namespace ColorsRGB
{
    const ColorRGB  DarkBlue  (0,       0.4470, 0.7410),
                    Orange    (0.8500,  0.3250, 0.0980),
                    DarkYellow(0.9290,  0.6940, 0.1250),
                    Purple    (0.4940,  0.1840, 0.5560),
                    Green     (0.4660,  0.6740, 0.1880),
                    LightBlue (0.3010,  0.7450, 0.9330),
                    DarkRed   (0.6350,  0.0780, 0.1840),
                    Black     (0.0,     0.0,    0.0),
                    White     (1.0,     1.0,    1.0),
                    Lime      (0.0,     1.0,    0.0),
                    Blue      (0.0,     0.0,    1.0),
                    Yellow    (1.0,     1.0,    0.0),
                    Cyan      (0.0,     1.0,    1.0),
                    Magenta   (1.0,     0.0,    1.0),
                    Silver    (0.75,    0.75,   0.75),
                    Gray      (0.5,     0.5,    0.5),
                    Maroon    (0.5,     0,      0),
                    Olive     (0.5,     0.5,    0),
                    Teal      (0,       0.5,    0.5),
                    Navy      (0.0,     0.0,    0.5);


    const ColorRGB color_arr[] = {DarkBlue, Orange, DarkYellow, Purple, Green, LightBlue, DarkRed, Black, White, Lime, Blue, Yellow, Cyan, Magenta, Silver, Gray, Maroon, Olive, Teal, Navy};
    int temp = ColorRGB::init();//TO-DO: Why does this not work with void. Bad style
};

int ColorRGB::init() 
{
  s_current_color = ColorsRGB::color_arr[0];
  s_current_id = 0;
  return 0;
}

void ColorRGB::cycle()
{
  if(s_current_id == s_max_id)
  {
    //if color palette is exhausted, generate random colors
    ColorRGB temp(((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX), ((double) rand() / RAND_MAX));
    s_current_color = temp;
  }
  else
  {
    s_current_id++;
    s_current_color = ColorsRGB::color_arr[s_current_id];
  }
}

ColorRGB ColorRGB::getCurrent()
{
  return s_current_color;
}