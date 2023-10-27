/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */

#include "utils/intpoint.h" // INT2MM
#include "GCodePathConfig.h"

namespace cura 
{

GCodePathConfig::GCodePathConfig(const GCodePathConfig& other)
: type(other.type)
, extruder_nr(other.extruder_nr)
, speed_derivatives(other.speed_derivatives)
, line_width(other.line_width)
, layer_thickness(other.layer_thickness)
, flow(other.flow)
, extrusion_mm3_per_mm(other.extrusion_mm3_per_mm)
, is_bridge_path(other.is_bridge_path)
, fan_speed(other.fan_speed)
, override_fan_speed(other.override_fan_speed)
, layer_z(other.layer_z)
{
}

GCodePathConfig::GCodePathConfig(const PrintFeatureType& type, const int extruder_nr, const coord_t line_width, const coord_t layer_height, double flow, GCodePathConfig::SpeedDerivatives speed_derivatives, bool is_bridge_path, double fan_speed, bool override_fan_speed, int z)
: type(type)
, extruder_nr(extruder_nr)
, speed_derivatives(speed_derivatives)
, line_width(line_width)
, layer_thickness(layer_height)
, flow(flow)
, extrusion_mm3_per_mm(calculateExtrusion())
, is_bridge_path(is_bridge_path)
, override_fan_speed(override_fan_speed)
, fan_speed(fan_speed)
, layer_z(z)
{
}

void GCodePathConfig::smoothSpeed(GCodePathConfig::SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer_nr) 
{
    double max_speed_layer = max_speed_layer_nr;
    speed_derivatives.speed = (speed_derivatives.speed * layer_nr) / max_speed_layer + (first_layer_config.speed * (max_speed_layer - layer_nr) / max_speed_layer);
    speed_derivatives.acceleration = (speed_derivatives.acceleration * layer_nr) / max_speed_layer + (first_layer_config.acceleration * (max_speed_layer - layer_nr) / max_speed_layer);
    speed_derivatives.jerk = (speed_derivatives.jerk * layer_nr) / max_speed_layer + (first_layer_config.jerk * (max_speed_layer - layer_nr) / max_speed_layer);
}

double GCodePathConfig::getExtrusionMM3perMM() const
{
    return extrusion_mm3_per_mm;
}

double GCodePathConfig::getSpeed() const
{
    return speed_derivatives.speed;
}

double GCodePathConfig::getAcceleration() const
{
    return speed_derivatives.acceleration;
}

double GCodePathConfig::getJerk() const
{
    return speed_derivatives.jerk;
}

int GCodePathConfig::getLineWidth() const
{
    return line_width;
}

int GCodePathConfig::getLayerThickness() const
{
    return this->layer_thickness;
}

const PrintFeatureType& GCodePathConfig::getPrintFeatureType() const
{
    return this->type;
}

bool GCodePathConfig::isTravelPath() const
{
    return line_width == 0;
}

bool GCodePathConfig::isBridgePath() const
{
    return is_bridge_path;
}

double GCodePathConfig::getFanSpeed() const
{
    return fan_speed;
}

void GCodePathConfig::setFanSpeed(const double _fan_speed)
{
    fan_speed = _fan_speed;
}

bool GCodePathConfig::getIsOverrideFanSpeed() const
{
    return override_fan_speed;
}

void GCodePathConfig::setOverrideFanSpeed(const bool _override_fan_speed)
{
    this->override_fan_speed = _override_fan_speed;
}

double GCodePathConfig::getFlowPercentage() const
{
    return flow;
}

double GCodePathConfig::calculateExtrusion() const
{
    return INT2MM(line_width) * INT2MM(layer_thickness) * double(flow);
}

void GCodePathConfig::setSpeedDerivatives(double speed, double acceleration, double jerk)
{
    speed_derivatives.speed = speed;
    speed_derivatives.acceleration = acceleration;
    speed_derivatives.jerk = jerk;
}

}//namespace cura
