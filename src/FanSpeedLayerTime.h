#ifndef FAN_SPEED_LAYER_TIME_H
#define FAN_SPEED_LAYER_TIME_H

#include "settings/settings.h"

namespace cura 
{

struct FanSpeedLayerTimeSettings 
{
public:
    double cool_min_layer_time;//The minimum time spent in a layer
    double cool_min_layer_time_fan_speed_max;//The layer time which sets the threshold between regular fan speed and maximum fan speed. Layers that print slower than this time use regular fan speed. For faster layers the fan speed gradually increases towards the maximum fan speed.
    double cool_fan_speed_0;   //The speed at which the fans spin at the start of the print
    double cool_fan_speed_min; //Regular Fan Speed
    double cool_fan_speed_max; //Maximum Fan Speed
    double cool_min_speed;     //The minimum print speed
    int cool_fan_full_layer;   //The layer at which the fans spin on regular fan speed
	bool  enable_auto_control;
};

} // namespace cura

#endif // FAN_SPEED_LAYER_TIME_H