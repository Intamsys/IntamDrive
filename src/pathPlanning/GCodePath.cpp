//Copyright (C) 2016 Ultimaker
//Released under terms of the AGPLv3 License

#include "GCodePath.h"

namespace cura
{
GCodePath::GCodePath(const GCodePathConfig& config, SpaceFillType space_fill_type, float flow, float width_factor, bool spiralize, double speed_factor) :
config(&config),
space_fill_type(space_fill_type),
flow(flow),
width_factor(width_factor),
speed_factor(speed_factor),
spiralize(spiralize)
{
    retract = false;
    perform_z_hop = false;
    perform_prime = false;
    points = std::vector<Point>();
    done = false;
    fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
    estimates = TimeMaterialEstimates();

    override_speed_parameter = false;
    override_speed = config.getSpeed();
    override_acceleration = config.getAcceleration();
    override_jerk = config.getJerk();

    override_line_width_parameter = false;
    override_line_width = config.getLineWidth();
}

bool GCodePath::isTravelPath()
{
    return config->isTravelPath();
}

double GCodePath::getExtrusionMM3perMM()
{
    return flow * width_factor * config->getExtrusionMM3perMM();
}

int GCodePath::getLineWidthForLayerView()
{
    return flow * width_factor * config->getLineWidth() * config->getFlowPercentage();
}

void GCodePath::setFanSpeed(double _fan_speed)
{
    this->fan_speed = _fan_speed;
}

double GCodePath::getFanSpeed() const
{
    return (fan_speed >= 0 && fan_speed <= 100) ? fan_speed : config->getFanSpeed();
}

void GCodePath::setOverrideSpeedParamater(const double speed, const double acceleration, const double jerk)
{
    if (std::abs(override_speed - speed) > 0.1 || std::abs(override_acceleration - acceleration) > 0.1 || std::abs(override_jerk - jerk) > 0.1)
    {
        override_speed_parameter = true;
        override_speed = speed;
        override_acceleration = acceleration;
        override_jerk = jerk;
    }
}

double GCodePath::getOverrideSpeed() const
{
    return override_speed;
}

double GCodePath::getOverrideAcceleration() const
{
    return override_acceleration;
}

double GCodePath::getOverrideJerk() const
{
    return override_jerk;
}

bool GCodePath::isOverrideSpeedParameter() const
{
    return override_speed_parameter;
}

bool GCodePath::isOverrideFanSpeedParameter() const
{
    return  this->override_fan_speed;
}

bool GCodePath::isOverrideLineWidthParameter() const
{
    return this->override_line_width_parameter;
}

int  GCodePath::getOverrideLineWidthParameter() const
{
    return this->override_line_width;
}

void GCodePath::setOverrideLineWidthParameter(const int line_width)
{
    if (override_line_width != line_width)
    {
        this->override_line_width_parameter = true;
        override_line_width = line_width;
    }
}

Point GCodePath::GetLastPoint()
{
    return points[points.size() - 1];
}

void GCodePath::combineCollinearPoints(const Point last_point)
{
    if (points.size() < 2)
        return;
    
    bool is_closed = (last_point == points[points.size() - 1]) ? true : false;
    int max_index = is_closed ? points.size() + 1 : points.size();

    for (int idx = 0; idx < points.size() - 1; ++idx)
    {
        Point ptPre;
        if (idx == 0)
        {
            ptPre = last_point;
        }
        else
        {
            ptPre = points[idx - 1];
        }

        Point ptCur = points[idx];
        Point ptNxt = points[idx + 1];
        
        //compute triangle area.
        int64_t a = ptPre.X * (ptCur.Y - ptNxt.Y) + ptCur.X * (ptNxt.Y - ptPre.Y) + ptNxt.X * (ptPre.Y - ptCur.Y);
        if (a == 0)
        {
            points.erase(points.begin() + idx);
            idx = idx - 1;
            if (idx < 0)
            {
                idx = 0;
            }
        }
    }
}

void GCodePath::combineNearestPoints(const Point last_point)
{
    if (points.size() < 2)
        return;

    bool is_closed = (last_point == points[points.size() - 1]) ? true : false;
    int max_index = is_closed ? points.size() + 1 : points.size();

    int linewidth = config->getLineWidth() / 4;
    for (int idx = 0; idx < points.size() - 2; ++idx)
    {
        Point ptCur = points[idx];
        Point ptNxt = points[idx + 1];
        if (vSize(ptNxt - ptCur) < linewidth)
        {
            points[idx].X = (ptCur.X + ptNxt.X) / 2;
            points[idx].Y = (ptCur.Y + ptNxt.Y) / 2;
            points.erase(points.begin() + idx + 1);
            idx = idx - 1;
            if (idx < 0)
            {
                idx = 0;
            }
            if (points.size() < 2)
                break;
        }
    }
}

void GCodePath::findExtremePoints(const GCodePath* prev_path, const GCodePath* next_path, const Point last_point, const double dbExtremeAngle)
{
    isExtremePtFlag.resize(points.size(), false);
    //第一个点
    if (prev_path == nullptr)
    {
        isExtremePtFlag[0] = true;
    }
    else
    {
        assert(last_point == prev_path->points[prev_path->points.size()-1]);
        Point ptPre = last_point;
        Point ptCur = points[0];
        if (points.size() == 1)
        {
            if (next_path == nullptr)
            {
                isExtremePtFlag[0] = true;
            }
            else if(this->config->getPrintFeatureType() == next_path->config->getPrintFeatureType())
            {
                Point ptNxt = next_path->points[0];
                if (angle(ptCur - ptPre, ptNxt - ptCur) > dbExtremeAngle)
                    isExtremePtFlag[0] = true;
                else
                    isExtremePtFlag[0] = false;
            }
            else
            {
                isExtremePtFlag[0] = true;
            }
        }
        else
        {
            Point ptNxt = points[1];
            if (angle(ptCur - ptPre, ptNxt - ptCur) > dbExtremeAngle)
                isExtremePtFlag[0] = true;
            else
                isExtremePtFlag[0] = false;
        }
    }

    for (unsigned int idx = 1; idx < points.size() - 1; ++idx)
    {
        Point ptPre = points[idx - 1];
        Point ptCur = points[idx];
        Point ptNxt = points[idx + 1];
        if (angle(ptCur - ptPre, ptNxt - ptCur) > dbExtremeAngle)
            isExtremePtFlag[idx] = true;
        else
            isExtremePtFlag[idx] = false;
    }

    if (points.size() > 1)
    {
        bool is_closed = (last_point == points[points.size()-1]) ? true : false;
        int max_index = is_closed ? points.size() + 1 : points.size();
        //最后一个点
        if (is_closed || next_path == nullptr)
        {
            isExtremePtFlag[points.size()-1] = true;
        }
        else if (this->config->getPrintFeatureType() == next_path->config->getPrintFeatureType())
        {
            Point ptPre = points[points.size()-2];
            Point ptCur = points[points.size()-1];
            Point ptNxt = next_path->points[0];
            if (angle(ptCur - ptPre, ptNxt - ptCur) > dbExtremeAngle)
                isExtremePtFlag[points.size()-1] = true;
            else
                isExtremePtFlag[points.size()-1] = false;
        }
        else
        {
            isExtremePtFlag[points.size()-1] = true;
        }
    }
}

double GCodePath::getLineSpeed(int segment_idx)
{
    return line_speed[segment_idx];
}

double GCodePath::getRevisedLineSpeed(int segment_idx)
{
    if (revised_line_speed.size() == 0 || segment_idx > revised_line_speed.size() - 1)
        return -1;
    return revised_line_speed[segment_idx];
}

}
