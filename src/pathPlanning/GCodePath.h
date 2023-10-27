//Copyright (C) 2017 Ultimaker
//Released under terms of the AGPLv3 License

#ifndef PATH_PLANNING_G_CODE_PATH_H
#define PATH_PLANNING_G_CODE_PATH_H

#include "../SpaceFillType.h"
#include "../GCodePathConfig.h"
#include "../utils/intpoint.h"

#include "TimeMaterialEstimates.h"

namespace cura 
{

/*!
 * A class for representing a planned path.
 * 
 * A path consists of several segments of the same type of movement: retracted travel, infill extrusion, etc.
 * 
 * This is a compact premature representation in which are line segments have the same config, i.e. the config of this path.
 * 
 * In the final representation (gcode) each line segment may have different properties, 
 * which are added when the generated GCodePaths are processed.
 */
class GCodePath
{
public:
    GCodePath();

    const GCodePathConfig* config; //!< The configuration settings of the path.
    SpaceFillType space_fill_type; //!< The type of space filling of which this path is a part
    float flow; //!< A type-independent flow configuration (used for wall overlap compensation)
    float width_factor;
    double speed_factor; //!< A speed factor that is multiplied with the travel speed. This factor can be used to change the travel speed.
    bool retract; //!< Whether the path is a move path preceded by a retraction move; whether the path is a retracted move path. 
    bool perform_z_hop; //!< Whether to perform a z_hop in this path, which is assumed to be a travel path.
    bool perform_prime; //!< Whether this path is preceded by a prime (blob)
    std::vector<Point> points; //!< The points constituting this path.
    bool done;//!< Path is finished, no more moves should be added, and a new path should be started instead of any appending done to this one.

    bool spiralize; //!< Whether to gradually increment the z position during the printing of this path. A sequence of spiralized paths should start at the given layer height and end in one layer higher.

    bool override_fan_speed;
    double fan_speed; //!< fan speed override for this path

    TimeMaterialEstimates estimates; //!< Naive time and material estimates

    bool override_speed_parameter;

    double override_speed;
    double override_acceleration;
    double override_jerk;

    bool override_line_width_parameter;
    int override_line_width;

    std::vector<bool> isExtremePtFlag; //!< is extreme point?
    std::vector<double> line_speed;
    std::vector<double> revised_line_speed;

    /*!
     * \brief Creates a new g-code path.
     *
     * \param config The line configuration to use when printing this path.
     * \param space_fill_type The type of space filling of which this path is a
     * part.
     * \param flow The flow rate to print this path with.
     * \param spiralize Gradually increment the z-coordinate while traversing
     * \param speed_factor The factor that the travel speed will be multiplied with
     * this path.
     */
    GCodePath(const GCodePathConfig& config, SpaceFillType space_fill_type, float flow, float width_factor, bool spiralize, double speed_factor = 1.0);

    /*!
     * Whether this config is the config of a travel path.
     * 
     * \return Whether this config is the config of a travel path.
     */
    bool isTravelPath();

    /*!
     * Get the material flow in mm^3 per mm traversed.
     * 
     * \warning Can only be called after the layer height has been set (which is done while writing the gcode!)
     * 
     * \return The flow
     */
    double getExtrusionMM3perMM();

    /*!
     * Get the actual line width (modulated by the flow)
     * \return the actual line width as shown in layer view
     */
    int getLineWidthForLayerView();

    /*!
     * Set fan_speed
     *
     * \param fan_speed the fan speed to use for this path
     */
    void setFanSpeed(double _fan_speed);

    /*!
     * Get the fan speed for this path
     * \return the value of fan_speed if it is in the range 0-100, otherwise the value from the config
     */
    double getFanSpeed() const;

    bool isOverrideFanSpeedParameter() const;

    void setOverrideSpeedParamater(const double speed, const double acceleration, const double jerk);

    double getOverrideSpeed() const;
    double getOverrideAcceleration() const;
    double getOverrideJerk() const;

    bool isOverrideSpeedParameter() const;

    bool isOverrideLineWidthParameter() const;

    int  getOverrideLineWidthParameter() const;

    void setOverrideLineWidthParameter(const int line_width);

    Point GetLastPoint();

    void combineCollinearPoints(const Point last_point);

    void combineNearestPoints(const Point last_point);

    void findExtremePoints(const GCodePath* prev_path, const GCodePath* next_path, const Point last_point, const double dbExtremeAngle);
    
    double getLineSpeed(int segment_idx);
    
    double getRevisedLineSpeed(int segment_idx);
};

}//namespace cura

#endif//PATH_PLANNING_G_CODE_PATH_H
