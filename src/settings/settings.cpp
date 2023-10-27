/** Copyright (C) 2016 Ultimaker - Released under terms of the AGPLv3 License */
#include <cctype>
#include <fstream>
#include <stdio.h>
#include <sstream> // ostringstream
#include <regex> // regex parsing for temp flow graph
#include <string> // stod (string to double)
#include "../utils/logoutput.h"

#include "settings.h"
#include "SettingRegistry.h"

#include "types/Angle.h"
#include "types/Duration.h" //For duration and time settings.
#include "types/LayerIndex.h" //For layer index settings.
#include "types/Ratio.h" //For ratio settings and percentages.
#include "types/Temperature.h" //For temperature settings.
#include "types/Velocity.h" //For velocity settings.

#include "../ExtruderTrain.h"
#include "../utils/logoutput.h"
#include "../utils/string.h" //For Escaped.
#include "../BeadingStrategy/BeadingStrategyFactory.h"

namespace cura
{
//c++11 no longer defines M_PI, so add our own constant.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

std::string toString(EGCodeFlavor flavor)
{
    switch (flavor)
    {
        case EGCodeFlavor::BFB:
            return "BFB";
        case EGCodeFlavor::MACH3:
            return "Mach3";
        case EGCodeFlavor::MAKERBOT:
            return "Makerbot";
        case EGCodeFlavor::ULTIGCODE:
            return "UltiGCode";
        case EGCodeFlavor::MARLIN_VOLUMATRIC:
            return "Marlin(Volumetric)";
        case EGCodeFlavor::GRIFFIN:
            return "Griffin";
        case EGCodeFlavor::REPETIER:
            return "Repetier";
        case EGCodeFlavor::REPRAP:
            return "RepRap";
        case EGCodeFlavor::MARLIN:
        default:
            return "Marlin";
    }
}

SettingsBaseVirtual::SettingsBaseVirtual()
: parent(nullptr)
{
}

SettingsBaseVirtual::SettingsBaseVirtual(SettingsBaseVirtual* parent)
: parent(parent)
{
}

SettingsBase::SettingsBase()
: SettingsBaseVirtual(nullptr)
{
}

SettingsBase::SettingsBase(SettingsBaseVirtual* parent)
: SettingsBaseVirtual(parent)
{
}

SettingsMessenger::SettingsMessenger(SettingsBaseVirtual* parent)
: SettingsBaseVirtual(parent)
{
}

void SettingsBase::_setSetting(std::string key, std::string value)
{
    setting_values[key] = value;
}

void SettingsBase::setSetting(std::string key, std::string value)
{
    if (SettingRegistry::getInstance()->settingExists(key))
    {
        _setSetting(key, value);
    }
    else
    {
        cura::logWarning("Setting an unregistered setting %s to %s\n", key.c_str(), value.c_str());
        _setSetting(key, value); // Handy when programmers are in the process of introducing a new setting
    }
}

void SettingsBase::setSettingInheritBase(std::string key, const SettingsBaseVirtual& parent)
{
    setting_inherit_base.emplace(key, &parent);
}

const std::string& SettingsBase::getSettingString(const std::string& key) const
{
    auto value_it = setting_values.find(key);
    if (value_it != setting_values.end())
    {
        return value_it->second;
    }
    auto inherit_override_it = setting_inherit_base.find(key);
    if (inherit_override_it != setting_inherit_base.end())
    {
        return inherit_override_it->second->getSettingString(key);
    }
    if (parent)
    {
        return parent->getSettingString(key);
    }

    cura::logError("Trying to retrieve unregistered setting with no value given: '%s'\n", key.c_str());
    std::exit(-1);
    static std::string empty_string; // use static object rather than "" to avoid compilation warning
    return empty_string;
}

void SettingsMessenger::setSetting(std::string key, std::string value)
{
    parent->setSetting(key, value);
}

void SettingsMessenger::setSettingInheritBase(std::string key, const SettingsBaseVirtual& new_parent)
{
    parent->setSettingInheritBase(key, new_parent);
}

const std::string& SettingsMessenger::getSettingString(const std::string& key) const
{
    return parent->getSettingString(key);
}

int SettingsBaseVirtual::getSettingAsIndex(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atoi(value.c_str());
}

int SettingsBaseVirtual::getSettingAsExtruderNr(std::string key) const
{
    int extruder_nr = getSettingAsIndex(key);
    if (extruder_nr == -1)
    {
        extruder_nr = getSettingAsIndex("extruder_nr");
    }
    const int max_extruders = getSettingAsCount("machine_extruder_count");
    if (extruder_nr >= max_extruders)
    {
        cura::logWarning("Trying to get extruder %s=%i, while there are only %i extruders.\n", key.c_str(), extruder_nr, max_extruders);
        return 0;
    }
    return extruder_nr;
}

int SettingsBaseVirtual::getSettingAsCount(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atoi(value.c_str());
}

unsigned int SettingsBaseVirtual::getSettingAsLayerNumber(std::string key) const
{
    const unsigned int indicated_layer_number = stoul(getSettingString(key));
    if (indicated_layer_number < 1) //Input checking: Layer 0 is not allowed.
    {
        cura::logWarning("Invalid layer number %i for setting %s.", indicated_layer_number, key.c_str());
        return 0; //Assume layer 1.
    }
    return indicated_layer_number - 1; //Input starts counting at layer 1, but engine code starts counting at layer 0.
}

double SettingsBaseVirtual::getSettingInMillimeters(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str());
}

coord_t SettingsBaseVirtual::getSettingInMicrons(std::string key) const
{
    return getSettingInMillimeters(key) * 1000.0;
}

double SettingsBaseVirtual::getSettingInAngleDegrees(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBaseVirtual::getSettingInAngleRadians(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str()) / 180.0 * M_PI;
}

bool SettingsBaseVirtual::getSettingBoolean(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "on")
        return true;
    if (value == "yes")
        return true;
    if (value == "true" || value == "True") //Python uses "True"
        return true;
    int num = atoi(value.c_str());
    return num != 0;
}

double SettingsBaseVirtual::getSettingInDegreeCelsius(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBaseVirtual::getSettingInMillimetersPerSecond(std::string key) const
{
    const std::string& value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingInCubicMillimeters(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str());
}

double SettingsBaseVirtual::getSettingInPercentage(std::string key) const
{
    const std::string& value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

double SettingsBaseVirtual::getSettingAsRatio(std::string key) const
{
    const std::string& value = getSettingString(key);
    return atof(value.c_str()) / 100.0;
}

double SettingsBaseVirtual::getSettingInSeconds(std::string key) const
{
    const std::string& value = getSettingString(key);
    return std::max(0.0, atof(value.c_str()));
}

DraftShieldHeightLimitation SettingsBaseVirtual::getSettingAsDraftShieldHeightLimitation(const std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "full")
    {
        return DraftShieldHeightLimitation::FULL;
    }
    else if (value == "limited")
    {
        return DraftShieldHeightLimitation::LIMITED;
    }
    return DraftShieldHeightLimitation::FULL; //Default.
}

FlowTempGraph SettingsBaseVirtual::getSettingAsFlowTempGraph(std::string key) const
{
    FlowTempGraph ret;
    std::string value_string = getSettingString(key);
    if (value_string.empty())
    {
        return ret; //Empty at this point.
    }
    std::regex regex("(\\[([^,\\[]*),([^,\\]]*)\\])");
    // match with:
    // - the last opening bracket '['
    // - then a bunch of characters until the first comma
    // - a comma
    // - a bunch of cahracters until the first closing bracket ']'
    // matches with any substring which looks like "[  124.512 , 124.1 ]"

    // default constructor = end-of-sequence:
    std::regex_token_iterator<std::string::iterator> rend;

    int submatches[] = { 1, 2, 3 }; // match whole pair, first number and second number of a pair
    std::regex_token_iterator<std::string::iterator> match_iter(value_string.begin(), value_string.end(), regex, submatches);
    while (match_iter != rend)
    {
        match_iter++; // match the whole pair
        if (match_iter == rend)
        {
            break;
        }
        std::string first_substring = *match_iter++;
        std::string second_substring = *match_iter++;
        try
        {
            double first = std::stod(first_substring);
            double second = std::stod(second_substring);
            ret.data.emplace_back(first, second);
        }
        catch (const std::invalid_argument& e)
        {
            logError("Couldn't read 2D graph element [%s,%s] in setting '%s'. Ignored.\n", first_substring.c_str(), second_substring.c_str(), key.c_str());
        }
    }
    return ret;
}

FMatrix3x3 SettingsBaseVirtual::getSettingAsPointMatrix(std::string key) const
{
    FMatrix3x3 ret;

    const std::string& value_string = getSettingString(key);
    if (value_string.empty())
    {
        return ret; // standard matrix ([1,0,0],[0,1,0],[0,0,1])
    }

    std::string num("([^,\\] ]*)"); // match with anything but the next ',' ']' or space  and capture the match
    std::ostringstream row; // match with "[num,num,num]" and ignore whitespace
    row << "\\s*\\[\\s*" << num << "\\s*,\\s*" << num << "\\s*,\\s*" << num << "\\s*\\]\\s*";

    std::ostringstream matrix; // match with "[row,row,row]" and ignore whitespace
    matrix << "\\s*\\[" << row.str() << "\\s*,\\s*" << row.str() << "\\s*,\\s*" << row.str() << "\\]\\s*";

    std::regex point_matrix_regex(matrix.str());
    std::cmatch sub_matches;    // same as std::match_results<const char*> cm;
    std::regex_match(value_string.c_str(), sub_matches, point_matrix_regex);

    if (sub_matches.size() != 10) // one match for the whole string
    {
        logWarning("Mesh transformation matrix could not be parsed!\n\tFormat should be [[f,f,f],[f,f,f],[f,f,f]] allowing whitespace anywhere in between.\n\tWhile what was given was \"%s\".\n", value_string.c_str());
        return ret; // standard matrix ([1,0,0],[0,1,0],[0,0,1])
    }

    unsigned int sub_match_idx = 1; // skip the first because the first submatch is the whole string
    for (unsigned int x = 0; x < 3; x++)
    {
        for (unsigned int y = 0; y < 3; y++)
        {
            std::sub_match<const char*> sub_match = sub_matches[sub_match_idx];
            ret.m[y][x] = strtod(std::string(sub_match.str()).c_str(), nullptr);
            sub_match_idx++;
        }
    }

    return ret;
}

EGCodeFlavor SettingsBaseVirtual::getSettingAsGCodeFlavor(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "Griffin")
        return EGCodeFlavor::GRIFFIN;
    else if (value == "UltiGCode")
        return EGCodeFlavor::ULTIGCODE;
    else if (value == "Makerbot")
        return EGCodeFlavor::MAKERBOT;
    else if (value == "BFB")
        return EGCodeFlavor::BFB;
    else if (value == "MACH3")
        return EGCodeFlavor::MACH3;
    else if (value == "RepRap (Volumetric)")
        return EGCodeFlavor::MARLIN_VOLUMATRIC;
    else if (value == "Repetier")
        return EGCodeFlavor::REPETIER;
    else if (value == "RepRap (RepRap)")
        return EGCodeFlavor::REPRAP;
    return EGCodeFlavor::MARLIN;
}

EFillMethod SettingsBaseVirtual::getSettingAsFillMethod(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "lines")
        return EFillMethod::LINES;
    if (value == "grid")
        return EFillMethod::GRID;
    if (value == "cubic")
        return EFillMethod::CUBIC;
    if (value == "cubicsubdiv")
        return EFillMethod::CUBICSUBDIV;
    if (value == "tetrahedral")
        return EFillMethod::TETRAHEDRAL;
    if (value == "quarter_cubic")
        return EFillMethod::QUARTER_CUBIC;
    if (value == "triangles")
        return EFillMethod::TRIANGLES;
    if (value == "trihexagon")
        return EFillMethod::TRIHEXAGON;
    if (value == "concentric")
        return EFillMethod::CONCENTRIC;
    if (value == "concentric_3d")
        return EFillMethod::CONCENTRIC_3D;
    if (value == "zigzag")
        return EFillMethod::ZIG_ZAG;
    if (value == "cross")
        return EFillMethod::CROSS;
    if (value == "cross_3d")
        return EFillMethod::CROSS_3D;
	if (value == "gyroid")
		return EFillMethod::GYROID;
    return EFillMethod::NONE;
}

EPlatformAdhesion SettingsBaseVirtual::getSettingAsPlatformAdhesion(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "brim")
        return EPlatformAdhesion::BRIM;
    if (value == "raft")
        return EPlatformAdhesion::RAFT;
    if (value == "none")
        return EPlatformAdhesion::NONE;
    return EPlatformAdhesion::SKIRT;
}

ESupportType SettingsBaseVirtual::getSettingAsSupportType(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "everywhere")
        return ESupportType::EVERYWHERE;
    if (value == "buildplate")
        return ESupportType::PLATFORM_ONLY;
    return ESupportType::NONE;
}

EZSeamType SettingsBaseVirtual::getSettingAsZSeamType(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "random")
        return EZSeamType::RANDOM;
    if (value == "shortest")
        return EZSeamType::SHORTEST;
    if (value == "back")
        return EZSeamType::USER_SPECIFIED;
    if (value == "sharpest_corner")
        return EZSeamType::SHARPEST_CORNER;
    return EZSeamType::SHORTEST;
}

EZSeamCornerPrefType SettingsBaseVirtual::getSettingAsZSeamCornerPrefType(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "z_seam_corner_none")
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE;
    if (value == "z_seam_corner_inner")
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER;
    if (value == "z_seam_corner_outer")
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER;
    if (value == "z_seam_corner_any")
        return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY;
    return EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE;
}

ESurfaceMode SettingsBaseVirtual::getSettingAsSurfaceMode(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "normal")
        return ESurfaceMode::NORMAL;
    if (value == "surface")
        return ESurfaceMode::SURFACE;
    if (value == "both")
        return ESurfaceMode::BOTH;
    return ESurfaceMode::NORMAL;
}

FillPerimeterGapMode SettingsBaseVirtual::getSettingAsFillPerimeterGapMode(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "nowhere")
    {
        return FillPerimeterGapMode::NOWHERE;
    }
    if (value == "everywhere")
    {
        return FillPerimeterGapMode::EVERYWHERE;
    }
    return FillPerimeterGapMode::NOWHERE;
}

BuildPlateShape SettingsBaseVirtual::getSettingAsBuildPlateShape(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "rectangular")
    {
        return BuildPlateShape::RECTANGULAR;
    }
    if (value == "elliptic")
    {
        return BuildPlateShape::ELLIPTIC;
    }
    return BuildPlateShape::RECTANGULAR;
}

SupportStructure SettingsBaseVirtual::getSettingAsSupportStructure(std::string key) const
{
    const std::string& value = getSettingString(key);
    if(value == "normal")
    {
        return SupportStructure::NORMAL;
    }
    if (value == "tree")
    {
        return SupportStructure::TREE;
    }
    return SupportStructure::NORMAL;
}

CombingMode SettingsBaseVirtual::getSettingAsCombingMode(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "off")
    {
        return CombingMode::OFF;
    }
    if (value == "all")
    {
        return CombingMode::ALL;
    }
    if (value == "noskin")
    {
        return CombingMode::NO_SKIN;
    }
    return CombingMode::ALL;
}

SupportDistPriority SettingsBaseVirtual::getSettingAsSupportDistPriority(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "xy_overrides_z")
    {
        return SupportDistPriority::XY_OVERRIDES_Z;
    }
    if (value == "z_overrides_xy")
    {
        return SupportDistPriority::Z_OVERRIDES_XY;
    }
    return SupportDistPriority::XY_OVERRIDES_Z;
}

SlicingTolerance SettingsBaseVirtual::getSettingAsSlicingTolerance(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "inclusive")
    {
        return SlicingTolerance::INCLUSIVE;
    }
    if (value == "exclusive")
    {
        return SlicingTolerance::EXCLUSIVE;
    }
    return SlicingTolerance::MIDDLE;
}

PrimeTowerType SettingsBaseVirtual::getSettingAsPrimeTowerType(std::string key) const
{
	const std::string& value = getSettingString(key);
	if (value == "nested")
	{
		return PrimeTowerType::NESTED;
	}
	return PrimeTowerType::INTERLACED;
}

PrimeTowerHeightType SettingsBaseVirtual::getSettingAsPrimeTowerHeightType(std::string key)const
{
	const std::string& value = getSettingString(key);
	if (value == "normal")
	{
		return PrimeTowerHeightType::NORMAL;
	}
	return PrimeTowerHeightType::FULL;
}

InsetDirection SettingsBaseVirtual::getSettingsAsInsetDirection(std::string key) const
{
    const std::string& value = getSettingString(key);
    if (value == "inside_out")
    {
        return InsetDirection::INSIDE_OUT;
    }
    if (value == "outside_in")
    {
        return InsetDirection::OUTSIDE_IN;
    }
    return InsetDirection::CENTER_LAST;
}

std::vector<int> SettingsBaseVirtual::getSettingAsIntegerList(std::string key) const
{
    std::vector<int> result;
    const std::string& value_string = getSettingString(key);
    if (!value_string.empty()) {
        // we're looking to match one or more integer values separated by commas and surrounded by square brackets
        // note that because the QML RegExpValidator only stops unrecognised characters being input
        // and doesn't actually barf if the trailing ] is missing, we are lenient here and make it optional
        std::regex list_contents_regex("\\[([^\\]]*)\\]?");
        std::smatch list_contents_match;
        if (std::regex_search(value_string, list_contents_match, list_contents_regex) && list_contents_match.size() > 1)
        {
            std::string elements = list_contents_match.str(1);
            std::regex element_regex("\\s*(-?[0-9]+)\\s*,?");
            // default constructor = end-of-sequence:
            std::regex_token_iterator<std::string::iterator> rend;

            std::regex_token_iterator<std::string::iterator> match_iter(elements.begin(), elements.end(), element_regex, 0);
            while (match_iter != rend)
            {
                std::string val = *match_iter++;
                try
                {
                    result.push_back(std::stoi(val));
                }
                catch (const std::invalid_argument& e)
                {
                    logError("Couldn't read integer value (%s) in setting '%s'. Ignored.\n", val.c_str(), key.c_str());
                }
            }
        }
    }
    return result;
}

Settings::Settings()
{
	parent = nullptr; //Needs to be properly initialised because we check against this if the parent is not set.
}

void Settings::add(const std::string& key, const std::string value)
{
	if (settings.find(key) != settings.end()) //Already exists.
	{
		settings[key] = value;
	}
	else //New setting.
	{
		settings.emplace(key, value);
	}
}

template<> std::string Settings::get<std::string>(const std::string& key) const
{
	//If this settings base has a setting value for it, look that up.
	if (settings.find(key) != settings.end())
	{
		return settings.at(key);
	}

	if (parent)
	{
		return parent->get<std::string>(key);
	}

	logError("Trying to retrieve setting with no value given: '%s'\n", key.c_str());
	std::exit(2);
}

template<> double Settings::get<double>(const std::string& key) const
{
	return atof(get<std::string>(key).c_str());
}

template<> size_t Settings::get<size_t>(const std::string& key) const
{
	return std::stoul(get<std::string>(key).c_str());
}

template<> int Settings::get<int>(const std::string& key) const
{
	return atoi(get<std::string>(key).c_str());
}

template<> bool Settings::get<bool>(const std::string& key) const
{
	const std::string& value = get<std::string>(key);
	if (value == "on" || value == "yes" || value == "true" || value == "True")
	{
		return true;
	}
	const int num = atoi(value.c_str());
	return num != 0;
}
template<> coord_t Settings::get<coord_t>(const std::string& key) const
{
	return MM2INT(get<double>(key)); //The settings are all in millimetres, but we need to interpret them as microns.
}

template<> AngleRadians Settings::get<AngleRadians>(const std::string& key) const
{
	return get<double>(key) * M_PI / 180; //The settings are all in degrees, but we need to interpret them as radians.
}

template<> AngleDegrees Settings::get<AngleDegrees>(const std::string& key) const
{
	return get<double>(key);
}

template<> Temperature Settings::get<Temperature>(const std::string& key) const
{
	return get<double>(key);
}

template<> Velocity Settings::get<Velocity>(const std::string& key) const
{
	return get<double>(key);
}

template<> Ratio Settings::get<Ratio>(const std::string& key) const
{
	return get<double>(key) / 100.0; //The settings are all in percentages, but we need to interpret them as radians.
}

template<> Duration Settings::get<Duration>(const std::string& key) const
{
	return get<double>(key);
}

template<> std::vector<double> Settings::get<std::vector<double>>(const std::string& key) const
{
	const std::string& value_string = get<std::string>(key);

	std::vector<double> result;
	if (value_string.empty())
	{
		return result;
	}

	/* We're looking to match one or more floating point values separated by
	* commas and surrounded by square brackets. Note that because the QML
	* RegExpValidator only stops unrecognised characters being input and
	* doesn't actually barf if the trailing ']' is missing, we are lenient here
	* and make that bracket optional.
	*/
	std::regex list_contents_regex(R"(\[([^\]]*)\]?)");
	std::smatch list_contents_match;
	if (std::regex_search(value_string, list_contents_match, list_contents_regex) && list_contents_match.size() > 1)
	{
		std::string elements = list_contents_match.str(1);
		std::regex element_regex(R"(\s*([+-]?[0-9]*\.?[0-9]+)\s*,?)");
		std::regex_token_iterator<std::string::iterator> rend; //Default constructor gets the end-of-sequence iterator.

		std::regex_token_iterator<std::string::iterator> match_iter(elements.begin(), elements.end(), element_regex, 0);
		while (match_iter != rend)
		{
			std::string value = *match_iter++;
			try
			{
				result.push_back(std::stod(value));
			}
			catch (const std::invalid_argument& e)
			{
				logError("Couldn't read floating point value (%s) in setting '%s'. Ignored.\n", value.c_str(), key.c_str());
			}
		}
	}
	return result;
}

template<> std::vector<int> Settings::get<std::vector<int>>(const std::string& key) const
{
	std::vector<double> values_doubles = get<std::vector<double>>(key);
	std::vector<int> values_ints;
	values_ints.reserve(values_doubles.size());
	for (double value : values_doubles)
	{
		values_ints.push_back(std::round(value)); //Round to nearest integer.
	}
	return values_ints;
}

template<> std::vector<AngleDegrees> Settings::get<std::vector<AngleDegrees>>(const std::string& key) const
{
	std::vector<double> values_doubles = get<std::vector<double>>(key);
	return std::vector<AngleDegrees>(values_doubles.begin(), values_doubles.end()); //Cast them to AngleDegrees.
}

const std::string Settings::getAllSettingsString() const
{
	std::stringstream sstream;
	for (const std::pair<std::string, std::string> pair : settings)
	{
		//char buffer[4096];
		//snprintf(buffer, 4096, " -s %s=\"%s\"", pair.first.c_str(), Escaped{ pair.second.c_str() }.str);
		//sstream << buffer;
	}
	return sstream.str();
}

bool Settings::has(const std::string& key) const
{
	return settings.find(key) != settings.end();
}

void Settings::setParent(Settings* new_parent)
{
	parent = new_parent;
}

std::string Settings::getWithoutLimiting(const std::string& key) const
{
	if (settings.find(key) != settings.end())
	{
		return settings.at(key);
	}
	else if (parent)
	{
		return parent->get<std::string>(key);
	}
	else
	{
		logError("Trying to retrieve setting with no value given: '%s'\n", key.c_str());
		std::exit(2);
	}
}



}//namespace cura

