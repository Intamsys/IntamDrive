//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <stdarg.h>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <fstream>
#include "gcodeExport.h"
#include "utils/logoutput.h"
#include "PrintFeature.h"
#include "utils/Date.h"
#include "utils/string.h" // MMtoStream, PrecisionedDouble
#include "sliceDataStorage.h"
#include "LayerPlan.h"
#include "Preheat.h"
namespace cura {

double layer_height; //!< report basic layer height in RepRap gcode file.

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, layer_nr(0)
, extruder_switch_time(0)
, extruder_switch_count(0)
{
    *output_stream << std::fixed;

    current_e_value = 0;
    current_extruder = 0;

    total_print_times = std::vector<double>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

    currentSpeed = 1;
    current_print_acceleration = -1;
    current_travel_acceleration = -1;
    current_jerk = -1;
    current_max_z_feedrate = -1;
	current_travel_speed = 1;

    isZHopped = 0;
    setFlavor(EGCodeFlavor::MARLIN);
    firmware_retract = false;
    initial_bed_temp = 0;

    extruder_count = 0;

    total_bounding_box = AABB3D();

	switch_first_start_Extruder = false;
	current_layer_z = 0;

    total_layers = 0;
    file_head_len = 0;

    build_plate_temperature = 0.0;
    chamber_temperature = 0.0;
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(const MeshGroup* meshgroup)
{
    setFlavor(meshgroup->getSettingAsGCodeFlavor("machine_gcode_flavor"));
    firmware_retract = meshgroup->getSettingBoolean("machine_firmware_retract");
    use_extruder_offset_to_offset_coords = meshgroup->getSettingBoolean("machine_use_extruder_offset_to_offset_coords");
    extruder_count = meshgroup->getSettingAsCount("machine_extruder_count");
	layer_height = meshgroup->getSettingInMicrons("layer_height");
	enable_prime_tower = meshgroup->getSettingBoolean("prime_tower_enable");
	prime_tower_high_type = meshgroup->getSettingAsPrimeTowerHeightType("prime_tower_height_type");

    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain* train = meshgroup->getExtruderTrain(extruder_nr);
        setFilamentDiameterAndDensity(extruder_nr, train->getSettingInMicrons("material_diameter"), train->getSettingInMicrons("material_density"));

        extruder_attr[extruder_nr].prime_pos = Point3(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"), train->getSettingInMicrons("extruder_prime_pos_z"));
        extruder_attr[extruder_nr].prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
        extruder_attr[extruder_nr].use_temp = train->getSettingBoolean("machine_nozzle_temp_enabled");
        extruder_attr[extruder_nr].is_prime_blob_enabled = train->getSettingBoolean("prime_blob_enable");

        extruder_attr[extruder_nr].nozzle_size = train->getSettingInMicrons("machine_nozzle_size");
        extruder_attr[extruder_nr].nozzle_offset = Point(train->getSettingInMicrons("machine_nozzle_offset_x"), train->getSettingInMicrons("machine_nozzle_offset_y"));
        extruder_attr[extruder_nr].material_guid = train->getSettingString("material_guid");
		extruder_attr[extruder_nr].material_name = train->getSettingString("material_name");
        extruder_attr[extruder_nr].material_key = train->getSettingString("material_key");

        extruder_attr[extruder_nr].start_code = train->getSettingString("machine_extruder_start_code");
        extruder_attr[extruder_nr].end_code = train->getSettingString("machine_extruder_end_code");

        extruder_attr[extruder_nr].last_retraction_prime_speed = train->getSettingInMillimetersPerSecond("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
        extruder_attr[extruder_nr].nozzle_id = train->getSettingString("machine_nozzle_id");
   
		currentFanSpeed[extruder_nr] = -1;

		enable_extruder_clean_up[extruder_nr] = train->getSettingBoolean("enable_extruder_interval_cleanup");
		extruder_clean_up_threshold[extruder_nr] = train->getSettingInMillimeters("max_interval_extrusion_value");
		extruder_clean_up_code[extruder_nr] = train->getSettingString("machine_extruder_clean_up_code");

		enable_extruder_layer_cleanup[extruder_nr] = train->getSettingBoolean("enable_extruder_layer_cleanup");
		interval_layer_num_cleanup[extruder_nr] = train->getSettingAsCount("interval_layer_num_cleanup");

        enable_cleanup_extruder_before_Skin_layer[extruder_nr] = train->getSettingBoolean("enable_clean_extruder_before_top_bottom_Skin_layer");

		accumulate_extrusion_value[extruder_nr] = 0.0;
		accumulate_layer_num_cleanup[extruder_nr] = 0.0;

		extruder_attr[extruder_nr].nozzle_offset_z = train->getSettingInMicrons("machine_nozzle_offset_z");
		extruder_attr[extruder_nr].z_offset_overlap = (int)(train->getSettingInMillimeters("z_offset_overlap_coefficient") * layer_height);
		enable_extruder_preheat[extruder_nr] = train->getSettingBoolean("enable_preheat");

        extruder_attr[extruder_nr].is_forcing_cooling_after_used = false;
        extruder_attr[extruder_nr].final_temp = train->getSettingInDegreeCelsius("material_print_temperature");
        extruder_attr[extruder_nr].print_temp = train->getSettingInDegreeCelsius("material_print_temperature");
	}

    machine_name = meshgroup->getSettingString("machine_name");
    machine_key  = meshgroup->getSettingString("machine_key");
    machine_buildplate_type = meshgroup->getSettingString("machine_buildplate_type");

	layer_height = meshgroup->getSettingInMillimeters("layer_height");

    relative_extrusion = meshgroup->getSettingBoolean("relative_extrusion");

    if (flavor == EGCodeFlavor::BFB)
    {
        new_line = "\r\n";
    }
    else 
    {
        new_line = "\n";
    }

    // initialize current_max_z_feedrate to firmware defaults
    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain* train = meshgroup->getExtruderTrain(extruder_nr);
        if (train->getSettingInMillimetersPerSecond("max_feedrate_z_override") <= 0.0)
        {
            // only initialize if firmware default for z feedrate is used by any extruder
            // that way we don't omit the M203 if Cura thinks the firmware is already at that feedrate, but it isn't the case
            current_max_z_feedrate = meshgroup->getSettingInMillimetersPerSecond("machine_max_feedrate_z");
            break;
        }
    }
    estimateCalculator.setFirmwareDefaults(meshgroup);
}

void GCodeExport::setInitialTemps(const MeshGroup& settings, const unsigned int start_extruder_nr)
{
    for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
    {
        const ExtruderTrain& train = *settings.getExtruderTrain(extr_nr);
        
        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
        double temp = (extr_nr == start_extruder_nr)? print_temp_here : train.getSettingInDegreeCelsius("material_standby_temperature");
        setInitialTemp(extr_nr, temp);
    }

    initial_bed_temp = settings.getSettingInDegreeCelsius("material_bed_temperature_layer_0");
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr[extruder_nr].initial_temp = temp;
    if (flavor == EGCodeFlavor::GRIFFIN || flavor == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr[extruder_nr].currentTemperature = temp;
    }
}

std::string GCodeExport::getFileHeader(const std::vector<bool>& extruder_is_used, const double* print_time, const std::vector<double>& filament_used, const std::vector<double>& filament_weight, const std::vector<std::string>& mat_names, const std::vector<std::string>& mat_keys)
{
    std::ostringstream prefix;
    switch (flavor)
    {
    case EGCodeFlavor::GRIFFIN:
	case EGCodeFlavor::MARLIN:
        prefix << ";START_OF_HEADER" << new_line;
        prefix << ";HEADER_VERSION:0.1" << new_line;
        prefix << ";FLAVOR:" << toString(flavor) << new_line;
        prefix << ";GENERATOR.NAME:Intam_Drive" << new_line;
        prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        prefix << ";TARGET_MACHINE.NAME:" << machine_name << new_line;
        prefix << ";TARGET_MACHINE.Key:" << machine_key << new_line;

        for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            if (!extruder_is_used[extr_nr]){
                continue;
            }
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr[extr_nr].initial_temp << new_line;
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
            }
            if (filament_used.size() == extruder_count)
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.WEIGHT_USED:" << static_cast<int>(filament_weight[extr_nr]) << new_line;
            }
            if (mat_names.size() == extruder_count && mat_names[extr_nr] != "")
            {
				prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.NAME:" << mat_names[extr_nr] << new_line;
            }
            if (mat_keys.size() == extruder_count && mat_keys[extr_nr] != "")
            {
                prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.KEY:" << mat_keys[extr_nr] << new_line;
            }
            
            const float nozzle_size = float(INT2MM(getNozzleSize(extr_nr)));
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << nozzle_size << new_line;
            prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.NAME:" << extruder_attr[extr_nr].nozzle_id << new_line;
        }

		if (print_time){
			unsigned int print_time_revised = static_cast<unsigned int>(*print_time);
			if (extruder_switch_time > 0)
			{
				print_time_revised += (unsigned int)(extruder_switch_time);
			}
			prefix << ";PRINT.TIME:" << print_time_revised << new_line;
		}

        if (total_bounding_box.min.x > total_bounding_box.max.x) //We haven't encountered any movement (yet). This probably means we're command-line slicing.
        {
            //Put some small default in there.
            total_bounding_box.min = Point3(1000, 1000, 1000);
            total_bounding_box.max = Point3(1, 1, 1);
        }
        prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box.min.x) << new_line;
        prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box.min.y) << new_line;
        prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box.min.z) << new_line;
        prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box.max.x) << new_line;
        prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box.max.y) << new_line;
        prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box.max.z) << new_line;
        prefix << ";END_OF_HEADER" << new_line;
        break;
    default:
        prefix << ";FLAVOR:" << toString(flavor) << new_line;
        prefix << ";TIME:" << ((print_time)? static_cast<int>(*print_time) : 6666) << new_line;
        if (flavor == EGCodeFlavor::ULTIGCODE)
        {
            prefix << ";MATERIAL:" << ((filament_used.size() >= 1)? static_cast<int>(filament_used[0]) : 6666) << new_line;
            prefix << ";MATERIAL2:" << ((filament_used.size() >= 2)? static_cast<int>(filament_used[1]) : 0) << new_line;

            prefix << ";NOZZLE_DIAMETER:" << float(INT2MM(getNozzleSize(0))) << new_line;
            // TODO: the second nozzle size isn't always initiated! ";NOZZLE_DIAMETER2:"
        }
        else if (flavor == EGCodeFlavor::REPRAP)
        {
            prefix << ";Filament used: " << ((filament_used.size() >= 1)? filament_used[0] / (1000 * extruder_attr[0].filament_area) : 0) << "m" << new_line;
            prefix << ";Layer height: " << layer_height << new_line;
        }
    }
    return prefix.str();
}

void GCodeExport::setLayerNr(unsigned int layer_nr_) 
{
    layer_nr = layer_nr_;
}

void GCodeExport::setPrimeTowerHeight(const SliceDataStorage& storage)
{
	prime_tower_height_layer = storage.max_print_height_second_to_last_extruder;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;    
    *output_stream << std::fixed;
}

bool GCodeExport::getExtruderIsUsed(const int extruder_nr) const
{
    assert(extruder_nr >= 0);
    assert(extruder_nr < MAX_EXTRUDERS);
    return extruder_attr[extruder_nr].is_used;
}

bool GCodeExport::getExtruderUsesTemp(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].use_temp;
}

int GCodeExport::getExtruderFinalTemp(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].final_temp;
}

int GCodeExport::getExtruderPrintTemp(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].print_temp;
}

int GCodeExport::getNozzleSize(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].nozzle_size;
}

Point GCodeExport::getExtruderOffset(const int id) const
{
    return extruder_attr[id].nozzle_offset;
}

std::string GCodeExport::getMaterialGUID(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].material_guid;
}

std::string GCodeExport::getMaterialNAME(const int extruder_nr) const
{
	return extruder_attr[extruder_nr].material_name;
}

std::string GCodeExport::getMaterialKEY(const int extruder_nr) const
{
    return extruder_attr[extruder_nr].material_key;
}

Point GCodeExport::getGcodePos(const int64_t x, const int64_t y, const int extruder_train) const
{
    if (use_extruder_offset_to_offset_coords) { return Point(x,y) - getExtruderOffset(extruder_train); }
    else { return Point(x,y); }
}

int GCodeExport::getGcodeZOffset(const int extruder_nr) const
{
	return extruder_attr[extruder_nr].nozzle_offset_z;
}

int GCodeExport::getGcodeZOverlap(const int extruder_nr) const
{
	return extruder_attr[extruder_nr].z_offset_overlap;
}

void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
    this->flavor = flavor;
    if (flavor == EGCodeFlavor::MACH3)
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruder_attr[n].extruderCharacter = 'A' + n;
    else
        for(int n=0; n<MAX_EXTRUDERS; n++)
            extruder_attr[n].extruderCharacter = 'E';
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
    {
        is_volumatric = true;
    }
    else
    {
        is_volumatric = false;
    }
}

EGCodeFlavor GCodeExport::getFlavor() const
{
    return this->flavor;
}

void GCodeExport::setZ(int z)
{
	current_layer_z = z;
}

int GCodeExport::getZ()
{
	return current_layer_z;
}

void GCodeExport::SetStartingExtruder(const size_t start_extruder)
{
	this->current_extruder = start_extruder;

	extruder_attr[start_extruder].is_used = true;
}

void GCodeExport::setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor)
{
    this->max_extrusion_offset = max_extrusion_offset;
    this->extrusion_offset_factor = extrusion_offset_factor;
}

Point3 GCodeExport::getPosition() const
{
    return currentPosition;
}
Point GCodeExport::getPositionXY() const
{
    return Point(currentPosition.x, currentPosition.y);
}

int GCodeExport::getPositionZ() const
{
    return currentPosition.z;
}

int GCodeExport::getExtruderNr() const
{
    return current_extruder;
}

void GCodeExport::setFilamentDiameterAndDensity(unsigned int extruder, int diameter, int density)
{
    double r = INT2MM(diameter) / 2.0;
    double area = M_PI * r * r;
    extruder_attr[extruder].filament_area = area;

    extruder_attr[extruder].filament_density = INT2MM(density);
}

double GCodeExport::getCurrentExtrudedVolume() const
{
    double extrusion_amount = current_e_value;
    if (!firmware_retract)
    { // no E values are changed to perform a retraction
        extrusion_amount -= extruder_attr[current_extruder].retraction_e_amount_at_e_start; // subtract the increment in E which was used for the first unretraction instead of extrusion
        extrusion_amount += extruder_attr[current_extruder].retraction_e_amount_current; // add the decrement in E which the filament is behind on extrusion due to the last retraction
    }
    if (is_volumatric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::eToMm(double e)
{
    if (is_volumatric)
    {
        return e / extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return e;
    }
}

double GCodeExport::mm3ToE(double mm3)
{
    if (is_volumatric)
    {
        return mm3;
    }
    else
    {
        return mm3 / extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::mmToE(double mm)
{
    if (is_volumatric)
    {
        return mm * extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return mm;
    }
}

double GCodeExport::getTotalFilamentUsed(int extruder_nr)
{
    if (extruder_nr == current_extruder)
        return extruder_attr[extruder_nr].totalFilament + getCurrentExtrudedVolume();
    return extruder_attr[extruder_nr].totalFilament;
}

double GCodeExport::getTotalFilamentWeight(int extruder_nr)
{
    double dbfilamentused = getTotalFilamentUsed(extruder_nr);
    return extruder_attr[extruder_nr].filament_density * dbfilamentused / 1000;
}

std::vector<double> GCodeExport::getTotalPrintTimePerFeature()
{
    return total_print_times;
}

double GCodeExport::getSumTotalPrintTimes()
{
    double sum = 0.0;
    for(double item : getTotalPrintTimePerFeature())
    {
        sum += item;
    }
    return sum;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    for(size_t i = 0; i < total_print_times.size(); i++)
    {
        total_print_times[i] = 0.0;
    }
    for(unsigned int e=0; e<MAX_EXTRUDERS; e++)
    {
        extruder_attr[e].totalFilament = 0.0;
        extruder_attr[e].currentTemperature = 0;
    }
    current_e_value = 0.0;
    estimateCalculator.reset();
}

void GCodeExport::updateTotalPrintTime()
{
    std::vector<double> estimates = estimateCalculator.calculate();
    for(size_t i = 0; i < estimates.size(); i++)
    {
        total_print_times[i] += estimates[i];
    }
    total_print_times[0] += estimateCalculator.getTime();
    estimateCalculator.reset();
    writeTimeComment(getSumTotalPrintTimes());
}

void GCodeExport::writeComment(std::string comment)
{
    *output_stream << ";";
    for(unsigned int i = 0; i < comment.length(); i++)
    {
        if (comment[i] == '\n')
        {
            *output_stream << "\\n";
        }else{
            *output_stream << comment[i];
        }
    }
    *output_stream << new_line;
}

void GCodeExport::writeTimeComment(const double time)
{
    *output_stream << ";TIME_ELAPSED:" << time << new_line;
}

void GCodeExport::writeTypeComment(PrintFeatureType type)
{
    switch (type)
    {
        case PrintFeatureType::OuterWall:
            *output_stream << ";TYPE:WALL-OUTER" << new_line;
            break;
        case PrintFeatureType::InnerWall:
            *output_stream << ";TYPE:WALL-INNER" << new_line;
            break;
        case PrintFeatureType::Skin:
            *output_stream << ";TYPE:SKIN" << new_line;
            break;
        case PrintFeatureType::Support:
		case PrintFeatureType::SupportInterface:
        case PrintFeatureType::SupportRoofInterface:
        case PrintFeatureType::SupportBottomInterface:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SkirtBrim:
            *output_stream << ";TYPE:SKIRT" << new_line;
            break;
        case PrintFeatureType::Infill:
            *output_stream << ";TYPE:FILL" << new_line;
            break;
        case PrintFeatureType::SupportInfill:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::MoveCombing:
        case PrintFeatureType::MoveRetraction:
        default:
            // do nothing
            break;
    }
}

void GCodeExport::writeLayerComment(int layer_nr)
{
    *output_stream << ";LAYER:" << layer_nr << new_line;
}

void GCodeExport::writeLayerCountComment(int layer_count)
{
    total_layers = layer_count;
    *output_stream << ";LAYER_COUNT:" << layer_count << new_line;
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << new_line;
}

void GCodeExport::writeExtrusionMode(bool set_relative_extrusion_mode)
{
    if (set_relative_extrusion_mode)
    {
        *output_stream << "M83 ;relative extrusion mode" << new_line;
    }
    else
    {
        *output_stream << "M82 ;absolute extrusion mode" << new_line;
    }
}

void GCodeExport::resetExtrusionValue()
{
    if (!relative_extrusion)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0" << new_line;
    }
    double current_extruded_volume = getCurrentExtrudedVolume();
    extruder_attr[current_extruder].totalFilament += current_extruded_volume;
    for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
    { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
        extruded_volume_at_retraction -= current_extruded_volume;
    }
    current_e_value = 0.0;
    extruder_attr[current_extruder].retraction_e_amount_at_e_start = extruder_attr[current_extruder].retraction_e_amount_current;
}

void GCodeExport::writeDelay(double timeAmount)
{
    *output_stream << "G4 P" << int(timeAmount * 1000) << new_line;
    estimateCalculator.addTime(timeAmount);
}

void GCodeExport::AddExtraTime(double timeAmount)
{
	estimateCalculator.addTime(timeAmount);
}

void GCodeExport::writeTravel(Point p, double speed, bool bforce)
{
    writeTravel(Point3(p.X, p.Y, current_layer_z), speed, bforce);
}
void GCodeExport::writeExtrusion(Point p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset, int z_offset)
{
    writeExtrusion(Point3(p.X, p.Y, current_layer_z - z_offset), speed, extrusion_mm3_per_mm, feature, update_extrusion_offset);
}

void GCodeExport::writeTravel(Point3 p, double speed, bool bforce)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z + isZHopped, speed, 0.0, PrintFeatureType::MoveCombing);
        return;
    }
    writeTravel(p.x, p.y, p.z + isZHopped, speed, bforce);
}

void GCodeExport::writeExtrusion(Point3 p, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature);
        return;
    }
    writeExtrusion(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature, update_extrusion_offset);
}

void GCodeExport::writeMoveBFB(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature) //Bits From Bytes GCode Flavor
{
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }
    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    float fspeed = speed * 60;
    float rpm = extrusion_per_mm * speed * 60;
    const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr[current_extruder].retraction_e_amount_current)
        {
            if (currentSpeed != double(rpm))
            {
                //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                //fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream << "M108 S" << PrecisionedDouble{1, rpm} << new_line;
                currentSpeed = double(rpm);
            }
            //Add M101 or M201 to enable the proper extruder.
            *output_stream << "M" << int((current_extruder + 1) * 100 + 1) << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
        }
        //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
        // (Trick copied from KISSlicer, thanks Jonathan)
        fspeed *= (rpm / (roundf(rpm * 100) / 100));

        //Increase the extrusion amount to calculate the amount of filament used.
        Point3 diff = Point3(x,y,z) - getPosition();
        
        current_e_value += extrusion_per_mm * diff.vSizeMM();
    }
    else
    {
        //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
        if (!extruder_attr[current_extruder].retraction_e_amount_current)
        {
            *output_stream << "M103" << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }
    *output_stream << "G1 X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y} << " Z" << MMtoStream{z};
    *output_stream << " F" << PrecisionedDouble{1, fspeed} << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed, feature);
}

void GCodeExport::writeTravel(int x, int y, int z, double speed, bool bforce)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z && !bforce)
        return;

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(400)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT

    const PrintFeatureType travel_move_type = extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
    const int display_width = extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1);
    CommandSocket::sendLineTo(travel_move_type, Point(x, y), display_width, layer_height, speed, currentFanSpeed[current_extruder]);

	*output_stream << "G0";
	writeFXYZE(speed, x, y, z, current_e_value, travel_move_type);

	current_travel_speed = speed;
}

void GCodeExport::writeExtrusion(int x, int y, int z, double speed, double extrusion_mm3_per_mm, PrintFeatureType feature, bool update_extrusion_offset)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
        return;

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(400)); // no crazy positions (this code should not be compiled for release)
    assert(extrusion_mm3_per_mm >= 0.0);
#endif //ASSERT_INSANE_OUTPUT

    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }

    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }
    if (extrusion_mm3_per_mm < 0.0)
    {
        logWarning("Warning! Negative extrusion move!\n");
    }

	double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);
	if (isZHopped > 0)
	{
		writeZhopEnd();
	}
	writeUnretractionAndPrime();

	Point3 diff = Point3(x, y, z) - currentPosition;
    //flow rate compensation
    double extrusion_offset = 0;
    if (diff.vSizeMM()) 
	{
        extrusion_offset = speed * extrusion_mm3_per_mm * extrusion_offset_factor;
        if (extrusion_offset > max_extrusion_offset) 
		{
            extrusion_offset = max_extrusion_offset;
        }
    }
    // write new value of extrusion_offset, which will be remembered.
    if ((update_extrusion_offset) && (extrusion_offset != current_e_offset)) 
	{
        current_e_offset = extrusion_offset;
        *output_stream << ";FLOW_RATE_COMPENSATED_OFFSET = " << current_e_offset << new_line;
    }

    double new_e_value = current_e_value + extrusion_per_mm * diff.vSizeMM();

	if (enable_extruder_clean_up[current_extruder])
	{
		accumulate_extrusion_value[current_extruder] += extrusion_per_mm * diff.vSizeMM();
	}

	if ((currentPosition.x != x || currentPosition.y != y) && currentPosition.z != z)
	{
		currentSpeed = current_max_z_feedrate;
		*output_stream << "G1 F" << PrecisionedDouble{ 1, current_max_z_feedrate * 60 } << " Z" << MMtoStream{ z } << new_line;
	}

    *output_stream << "G1";
    writeFXYZE(speed, x, y, z, new_e_value, feature);
}

void GCodeExport::writeFXYZE(double speed, int x, int y, int z, double e, PrintFeatureType feature, bool force_clean_up_extruder)
{
	if (currentSpeed != speed)
    {
        *output_stream << " F" << PrecisionedDouble{1, speed * 60};
        currentSpeed = speed;
    }

    Point gcode_pos = getGcodePos(x, y, current_extruder);

    *output_stream << " X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y};
    if (z != currentPosition.z)
    {
        *output_stream << " Z" << MMtoStream{z};
    }
    if (e + current_e_offset != current_e_value)
    {
        const double output_e = (relative_extrusion)? e + current_e_offset - current_e_value : e + current_e_offset;
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{4, output_e};

        total_bounding_box.include(Point3(gcode_pos.X, gcode_pos.Y, z));
    }

    *output_stream << new_line;

    currentPosition = Point3(x, y, z);
    current_e_value = e;
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(e)), speed, feature);
}

void GCodeExport::writeUnretractionAndPrime()
{
    const double prime_volume = extruder_attr[current_extruder].prime_volume;
    const double prime_volume_e = mm3ToE(prime_volume);
    current_e_value += prime_volume_e;
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        if (firmware_retract)
        { // note that BFB is handled differently
            *output_stream << "G11" << new_line;
            //Assume default UM2 retraction settings.
            if (prime_volume != 0)
            {
                *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{4, current_e_value} << new_line;
                currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            }
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0, PrintFeatureType::MoveRetraction);
        }
        else
        {
			current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
            const double output_e = (relative_extrusion)? extruder_attr[current_extruder].retraction_e_amount_current + prime_volume_e : current_e_value;
			
			*output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{4, output_e} << new_line;						
            
			currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        }
        if (getCurrentExtrudedVolume() > 10000.0 && flavor != EGCodeFlavor::BFB && flavor != EGCodeFlavor::MAKERBOT) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
        {
            resetExtrusionValue();
        }
        extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
    }
    else if (prime_volume != 0.0)
    {
        const double output_e = (relative_extrusion)? prime_volume_e : current_e_value;
        *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter;
        *output_stream << PrecisionedDouble{4, output_e} << new_line;
        currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::NoneType);
    }
    extruder_attr[current_extruder].prime_volume = 0.0;
}

void GCodeExport::writeRetraction(const RetractionConfig& config, bool force, bool extruder_switch)
{
    ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];

    if (flavor == EGCodeFlavor::BFB)//BitsFromBytes does automatic retraction.
    {
        if (extruder_switch)
        {
            if (!extr_attr.retraction_e_amount_current)
                *output_stream << "M103" << new_line;

            extr_attr.retraction_e_amount_current = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        }
        return;
    }

	double old_retraction_e_amount = extr_attr.retraction_e_amount_current;
    double new_retraction_e_amount = mmToE(config.distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;

    if (std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;   
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
        while (int(extruded_volume_at_previous_n_retractions.size()) > config.retraction_count_max && !extruded_volume_at_previous_n_retractions.empty()) 
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if (!force && config.retraction_count_max <= 0)
        {
            return;
        }
        if (!force && int(extruded_volume_at_previous_n_retractions.size()) == config.retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area) 
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if (int(extruded_volume_at_previous_n_retractions.size()) == config.retraction_count_max + 1) 
        {
            extruded_volume_at_previous_n_retractions.pop_back();
        }
    }

    if (firmware_retract)
    {
        if (extruder_switch && extr_attr.retraction_e_amount_current) 
        {
            return; 
        }
        *output_stream << "G10";
        if (extruder_switch)
        {
            *output_stream << " S1";
        }
        *output_stream << new_line;
        //Assume default UM2 retraction settings.
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value + retraction_diff_e_amount)), 25, PrintFeatureType::MoveRetraction); // TODO: hardcoded values!
    }
    else
    {	
        double speed = ((retraction_diff_e_amount < 0.0)? config.speed : extr_attr.last_retraction_prime_speed) * 60;
        current_e_value += retraction_diff_e_amount;
        const double output_e = (relative_extrusion)? retraction_diff_e_amount : current_e_value;

		* output_stream << "G1 F" << PrecisionedDouble{ 1, speed } << " " << extr_attr.extruderCharacter << PrecisionedDouble{ 4, output_e } << new_line;

        currentSpeed = speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction);
        extr_attr.last_retraction_prime_speed = config.primeSpeed;
    }

    extr_attr.retraction_e_amount_current = new_retraction_e_amount;
	extr_attr.prime_volume += config.prime_volume;
}

void GCodeExport::writeZhopStart(int hop_height)
{
    if (hop_height > 0)
    {
        isZHopped = hop_height;
        currentSpeed = current_max_z_feedrate;
        *output_stream << "G1 F" << PrecisionedDouble{1, current_max_z_feedrate * 60} << " Z" << MMtoStream{current_layer_z + isZHopped} << new_line;
        total_bounding_box.includeZ(current_layer_z + isZHopped);

		currentPosition.z = current_layer_z + isZHopped;

        assert(current_max_z_feedrate > 0.0 && "Z feedrate should be positive");
    }
}

void GCodeExport::writeZhopEnd()
{
    if (isZHopped)
    {
        isZHopped = 0;
        currentPosition.z = current_layer_z;
        currentSpeed = current_max_z_feedrate;
        *output_stream << "G1 F" << PrecisionedDouble{1, current_max_z_feedrate * 60} << " Z" << MMtoStream{current_layer_z} << new_line;

		currentPosition.z = current_layer_z;

        assert(current_max_z_feedrate > 0.0 && "Z feedrate should be positive");
    }
}

void GCodeExport::startExtruder(int new_extruder)
{
	if (!extruder_attr[new_extruder].is_used)	switch_first_start_Extruder = true;
    extruder_attr[new_extruder].is_used = true;
    if (new_extruder != current_extruder) // wouldn't be the case on the very first extruder start if it's extruder 0
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
        {
            *output_stream << "M135 T" << new_extruder << new_line;
        }
        else
        {
            *output_stream << "T" << new_extruder << new_line;
        }
    }

    current_extruder = new_extruder;

    assert(getCurrentExtrudedVolume() == 0.0 && "Just after an extruder switch we haven't extruded anything yet!");
    resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

    CommandSocket::setExtruderForSend(new_extruder);
    CommandSocket::setSendCurrentPosition( getPositionXY() );

    //Change the Z position so it gets re-written again. We do not know if the switch code modified the Z position.
    currentPosition.z += 1;
}

void GCodeExport::switchExtruder(const int new_extruder, const RetractionConfig& retraction_config_old_extruder)
{
    if (current_extruder == new_extruder)
        return;
	extruder_switch_count++;
		
    bool force = true;
    bool extruder_switch = true;
    writeRetraction(retraction_config_old_extruder, force, extruder_switch);

    resetExtrusionValue(); // zero the E value on the old extruder, so that the current_e_value is registered on the old extruder

    int old_extruder = current_extruder;

    const char *end_code = extruder_attr[old_extruder].end_code.c_str();

    if (*end_code)
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
        }

        writeCode(end_code);

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }
	writeFanCommand(0, old_extruder);

    startExtruder(new_extruder);
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream << str << new_line;
}

void GCodeExport:: writePrimeTrain(double travel_speed)
{
    if (extruder_attr[current_extruder].is_primed)
    { // extruder is already primed once!
        return;
    }
    if (extruder_attr[current_extruder].is_prime_blob_enabled)
    { // only move to prime position if we do a blob/poop
        // ideally the prime position would be respected whether we do a blob or not,
        // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
        // which is needed to automatically ignore the prime position for the UM3 machine when blob is disabled
        Point3 prime_pos = extruder_attr[current_extruder].prime_pos;
        if (!extruder_attr[current_extruder].prime_pos_is_abs)
        {
            prime_pos += currentPosition;
        }
        writeTravel(prime_pos, travel_speed);
    }

    if (flavor == EGCodeFlavor::GRIFFIN)
    {
        std::string command = "G280";
        if (!extruder_attr[current_extruder].is_prime_blob_enabled)
        {
            command += " S1";  // use S1 to disable prime blob
        }
        *output_stream << command << new_line;
    }
    else
    {
        // there is no prime gcode for other firmware versions...
    }

    extruder_attr[current_extruder].is_primed = true;
}

void GCodeExport::writeFanCommand(double speed, int extruder)
{
	if (std::abs(currentFanSpeed[extruder] - speed) < 0.1)
	{
		return;
	}

    if (speed > 0)
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M126 T0" << new_line; //value = speed * 255 / 100 // Makerbot cannot set fan speed...;
        else
        {
            *output_stream << "M106 T" << extruder << " S" << PrecisionedDouble{ 1, speed * 255 / 100 } << new_line;
        }
    }
    else
    {
        if (flavor == EGCodeFlavor::MAKERBOT)
            *output_stream << "M127 T0" << new_line;
        else
        {
            *output_stream << "M107 T" << extruder << new_line;
        }
    }
    currentFanSpeed[extruder] = speed;
}

void GCodeExport::writeTemperatureCommand(int extruder, double temperature, bool wait, bool startup, bool isPreheat)
{
    if (!extruder_attr[extruder].use_temp)
    {
        return;
    }
    if (temperature < 0)
    {
        return;
    }
    if (!wait && extruder_attr[extruder].currentTemperature == temperature)
    {
        return;
    }

    if (wait && extruder == lastest_extruder_heating_status.extruder_nr && lastest_extruder_heating_status.temperature == temperature)
    {
        return;
    }

	if (wait)
	{
		*output_stream << "M109";
	}
	else
	{
		*output_stream << "M104";
	}
	{
		*output_stream << " T" << extruder;
	}
#ifdef ASSERT_INSANE_OUTPUT
    assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
    *output_stream << " S" << PrecisionedDouble{1, temperature} << new_line;
    extruder_attr[extruder].currentTemperature = temperature;

    if (wait)
    {
        lastest_extruder_heating_status.heatandwaiting = true;
        lastest_extruder_heating_status.extruder_nr = extruder;
        lastest_extruder_heating_status.temperature = temperature;
    }
}

void GCodeExport::writeBedTemperatureCommand(double temperature, bool wait)
{
    if (flavor == EGCodeFlavor::ULTIGCODE)
    { // The UM2 family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    if (wait)
        *output_stream << "M190 S";
    else
        *output_stream << "M140 S";
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writePrintAcceleration(double acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M201 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 P" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 S" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
    }
    current_print_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeTravelAcceleration(double acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M202 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M204 T" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            writePrintAcceleration(acceleration);
            break;
    }
    current_travel_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeJerk(double jerk)
{
    if (current_jerk != jerk)
    {
        switch (getFlavor())
        {
            case EGCodeFlavor::REPETIER:
                *output_stream << "M207 X" << PrecisionedDouble{2, jerk} << new_line;
                break;
            case EGCodeFlavor::REPRAP:
                *output_stream << "M566 X" << PrecisionedDouble{2, jerk * 60} << " Y" << PrecisionedDouble{2, jerk * 60} << new_line;
                break;
            default:
                *output_stream << "M205 X" << PrecisionedDouble{2, jerk} << " Y" << PrecisionedDouble{2, jerk} << new_line;
                break;
        }
        current_jerk = jerk;
        estimateCalculator.setMaxXyJerk(jerk);
    }
}

void GCodeExport::writeMaxZFeedrate(double max_z_feedrate)
{
    if (current_max_z_feedrate != max_z_feedrate)
    {
        if (getFlavor() == EGCodeFlavor::REPRAP)
        {
            *output_stream << "M203 Z" << PrecisionedDouble{2, max_z_feedrate * 60} << new_line;
        }
        else if (getFlavor() != EGCodeFlavor::REPETIER) //Repetier firmware changes the "temperature monitor" to 0 when encountering a M203 command, which is undesired.
        {
            *output_stream << "M203 Z" << PrecisionedDouble{2, max_z_feedrate} << new_line;
        }
        current_max_z_feedrate = max_z_feedrate;
        estimateCalculator.setMaxZFeedrate(max_z_feedrate);
    }
}

double GCodeExport::getCurrentMaxZFeedrate()
{
    return current_max_z_feedrate;
}

void GCodeExport::finalize(const char* endCode)
{
    writeFanCommand(0, current_extruder);
    writeCode(endCode);
    int64_t print_time = getSumTotalPrintTimes();
    int mat_0 = getTotalFilamentUsed(0);
    log("Print time: %d\n", print_time);
    log("Print time (readable): %dh %dm %ds\n", print_time / 60 / 60, (print_time / 60) % 60, print_time % 60);
    log("Filament: %d\n", mat_0);
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));
    output_stream->flush();
}

void GCodeExport::writeBuildplateT(double bedTemperature)
{
	if (flavor == EGCodeFlavor::ULTIGCODE)
	{
		return;
	}
	std::ostringstream bedTmp;

	bedTmp << "M140 S" << bedTemperature;
	writeLine(bedTmp.str().c_str());
}

void GCodeExport::TK_ExtraE_Brush()
{
	const char *start_code = extruder_attr[current_extruder].start_code.c_str();

	if (*start_code)
	{
		if (relative_extrusion)
		{
			writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
		}

		writeCode(start_code);

		float current_extruder_retraction_value = 0;

		accumulate_extrusion_value[current_extruder] = 0;

		accumulate_layer_num_cleanup[current_extruder] = 0;

		if (relative_extrusion)
		{
			writeExtrusionMode(true); // restore relative extrusion mode
		}

		if (switch_first_start_Extruder && current_extruder_retraction_value<0)
		{
			*output_stream << "G1 F" << PrecisionedDouble{ 1, extruder_attr[current_extruder].last_retraction_prime_speed * 60 } << " E" << PrecisionedDouble{ 4, std::fabs(current_extruder_retraction_value) } << new_line;
			*output_stream << "G92 E0" << new_line;
		}
        switch_first_start_Extruder = false;
	}
}

void GCodeExport::setRetraction(const int extruder_nr, const RetractionConfig& config)
{
	extruder_attr[extruder_nr].retraction_e_amount_current = config.distance;
	switch_first_start_Extruder = false;
}

void GCodeExport::setExtruderRetractionConfig(std::vector<RetractionConfig>& extruder_retraction_config)
{
	retraction_config_per_extruder = extruder_retraction_config;
}

double GCodeExport::ComputeExtruderTemperature(const int extruder_nr, Preheat& preheat, const double dbTemp, const double dbTime, const bool isHeatupFlag)
{
    double dbRetTemp = 0;
    if (isHeatupFlag)
    {
        dbRetTemp = dbTemp + dbTime * preheat.getHeatupSpeed(extruder_nr, false);
        if (dbRetTemp > preheat.getInitialPrintTemp(extruder_nr))
            dbRetTemp = preheat.getInitialPrintTemp(extruder_nr);
    }
    else
    {
        dbRetTemp = dbTemp - dbTime * preheat.getCooldownSpeed(extruder_nr, false);
        if (dbRetTemp < preheat.getStandbyTemp(extruder_nr))
            dbRetTemp = preheat.getStandbyTemp(extruder_nr);
    }
    return dbRetTemp;
}

void GCodeExport::updateFileHeaderInfo(const std::vector<bool>& extruder_is_used, const double* print_time, const std::vector<double>& filament_used, const std::vector<double>& filament_weights, const std::vector<std::string>& mat_names, const std::vector<std::string>& mat_keys)
{
    std::string prefix = getFileHeaderInfo(extruder_is_used, print_time, filament_used, filament_weights, mat_names, mat_keys);
    std::streambuf* buf = output_stream->rdbuf();
    buf->pubseekpos(0, output_stream->beg);
    buf->sputn(prefix.c_str(), file_head_len);
    output_stream->flush();
}
std::string GCodeExport::getFileHeaderInfo(const std::vector<bool>& extruder_is_used, const double* print_time, const std::vector<double>& filament_used, const std::vector<double>& filament_weight, const std::vector<std::string>& mat_names, const std::vector<std::string>& mat_keys)
{
    file_head_len = 0;
    std::ostringstream default_prefix;
    switch (flavor)
    {
    case EGCodeFlavor::GRIFFIN:
    case EGCodeFlavor::MARLIN:
        default_prefix << ";START_OF_HEADER" << new_line;
        default_prefix << ";FLAVOR:" << toString(flavor) << new_line;
        default_prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        default_prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        default_prefix << ";TARGET_MACHINE.NAME:" << machine_name << new_line;
        default_prefix << ";TARGET_MACHINE.KEY:" << machine_key << new_line;
        for (unsigned int extr_nr = 0; extr_nr < extruder_count; extr_nr++)
        {
            if (!extruder_is_used[extr_nr])
            {
                continue;
            }
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << floatconvertstring(extruder_attr[extr_nr].initial_temp, 7) << new_line;
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << floatconvertstring(filament_used[extr_nr], 10) << new_line;
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.WEIGHT_USED:" << floatconvertstring(filament_weight[extr_nr], 10) << new_line;
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.NAME:" << mat_names[extr_nr] << new_line;
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.KEY:" << mat_keys[extr_nr] << new_line;
            const float nozzle_size = float(INT2MM(getNozzleSize(extr_nr)));
            default_prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << floatconvertstring(nozzle_size, 4) << new_line;
        }
        default_prefix << ";PRINT.BUILD_TEMPERATURE:" << floatconvertstring(build_plate_temperature, 7) << new_line;
        default_prefix << ";PRINT.CHAMBER_TEMPERATURE:" << floatconvertstring(chamber_temperature, 7) << new_line;

        if (print_time)
        {
            unsigned int print_time_revised = static_cast<unsigned int>(*print_time);

            if (extruder_switch_time > 0)
            {
                print_time_revised += (unsigned int)(extruder_switch_time);
            }
            default_prefix << ";PRINT.TIME:" << floatconvertstring(print_time_revised, 10) << new_line;
        }

        if (total_bounding_box.min.x > total_bounding_box.max.x)
        {
            total_bounding_box.min = Point3(1000000, 1000000, 1000000);
            total_bounding_box.max = Point3(1, 1, 1);
        }
        default_prefix << ";PRINT.SIZE.MIN.X:" << floatconvertstring(INT2MM(total_bounding_box.min.x), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MIN.Y:" << floatconvertstring(INT2MM(total_bounding_box.min.y), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MIN.Z:" << floatconvertstring(INT2MM(total_bounding_box.min.z), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.X:" << floatconvertstring(INT2MM(total_bounding_box.max.x), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.Y:" << floatconvertstring(INT2MM(total_bounding_box.max.y), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.Z:" << floatconvertstring(INT2MM(total_bounding_box.max.z), 7) << new_line;

        default_prefix << ";PRINT.TOTAL.LAYERS:" << floatconvertstring(total_layers, 7) << new_line;

        default_prefix << ";PRINT_TIME.LINE_TYPE.WALL-OUTER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::OuterWall)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.WALL-INNER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::InnerWall)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.FILL:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Infill)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SKIN:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Skin)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SUPPORT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Support)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SUPPORT-INTERFACE:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::SupportInterface)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.RAFT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Raft)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SKIRT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::SkirtBrim)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.PRIME-TOWER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::PrimeTower)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.TRAVEL:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::MoveCombing)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.RETRACTION:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::MoveRetraction)], 10) << new_line;
        default_prefix << ";END_OF_HEADER" << new_line;
        break;
    default:
        default_prefix << ";FLAVOR:UltiGCode" << new_line;
        if (print_time)
        {
            unsigned int print_time_revised = static_cast<unsigned int>(*print_time);
            default_prefix << ";TIME:" << floatconvertstring(print_time_revised, 10) << new_line;
        }
       
        default_prefix << ";MATERIAL:" << floatconvertstring(filament_used[0], 10) << new_line;
        default_prefix << ";MATERIAL2:" << 0 << new_line;
        default_prefix << ";START_OF_HEADER" << new_line;
        default_prefix << ";FLAVOR:SD CARD" << new_line;
        default_prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
        default_prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
        default_prefix << ";TARGET_MACHINE.NAME:" << machine_name << new_line;
        default_prefix << ";TARGET_MACHINE.KEY:" << machine_key << new_line;

        default_prefix << ";EXTRUDER_TRAIN.0.INITIAL_TEMPERATURE:" << floatconvertstring(extruder_attr[0].initial_temp, 7) << new_line;
        default_prefix << ";EXTRUDER_TRAIN.0.MATERIAL.VOLUME_USED:" << floatconvertstring(filament_used[0], 10) << new_line;
        default_prefix << ";EXTRUDER_TRAIN.0.MATERIAL.WEIGHT_USED:" << floatconvertstring(filament_weight[0], 10) << new_line;
        default_prefix << ";EXTRUDER_TRAIN.0.MATERIAL.NAME:" << mat_names[0] << new_line;
        default_prefix << ";EXTRUDER_TRAIN.0.MATERIAL.KEY:" <<  mat_keys[0] << new_line;
        const float nozzle_size = float(INT2MM(getNozzleSize(0)));
        default_prefix << ";EXTRUDER_TRAIN.0.NOZZLE.DIAMETER:" << floatconvertstring(nozzle_size, 4) << new_line;

        default_prefix << ";PRINT.BUILD_TEMPERATURE:" << floatconvertstring(build_plate_temperature, 7) << new_line;
        default_prefix << ";PRINT.CHAMBER_TEMPERATURE:" << floatconvertstring(chamber_temperature, 7) << new_line;

        if (print_time)
        {
            unsigned int print_time_revised = static_cast<unsigned int>(*print_time);
            default_prefix << ";PRINT.TIME:" << floatconvertstring(print_time_revised, 10) << new_line;
        }

        if (total_bounding_box.min.x > total_bounding_box.max.x)
        {
            total_bounding_box.min = Point3(1000000, 1000000, 1000000);
            total_bounding_box.max = Point3(1, 1, 1);
        }
        default_prefix << ";PRINT.SIZE.MIN.X:" << floatconvertstring(INT2MM(total_bounding_box.min.x), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MIN.Y:" << floatconvertstring(INT2MM(total_bounding_box.min.y), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MIN.Z:" << floatconvertstring(INT2MM(total_bounding_box.min.z), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.X:" << floatconvertstring(INT2MM(total_bounding_box.max.x), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.Y:" << floatconvertstring(INT2MM(total_bounding_box.max.y), 7) << new_line;
        default_prefix << ";PRINT.SIZE.MAX.Z:" << floatconvertstring(INT2MM(total_bounding_box.max.z), 7) << new_line;
        default_prefix << ";PRINT.TOTAL.LAYERS:" << floatconvertstring(total_layers, 7) << new_line;

        default_prefix << ";PRINT_TIME.LINE_TYPE.WALL-OUTER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::OuterWall)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.WALL-INNER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::InnerWall)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.FILL:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Infill)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SKIN:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Skin)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SUPPORT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Support)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SUPPORT-INTERFACE:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::SupportInterface)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.RAFT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::Raft)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.SKIRT:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::SkirtBrim)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.PRIME-TOWER:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::PrimeTower)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.TRAVEL:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::MoveCombing)], 10) << new_line;
        default_prefix << ";PRINT_TIME.LINE_TYPE.RETRACTION:" << floatconvertstring(total_print_times[static_cast<unsigned char>(PrintFeatureType::MoveRetraction)], 10) << new_line;
        default_prefix << ";END_OF_HEADER" << new_line;
        break;
    }
    file_head_len = default_prefix.str().length();
    return default_prefix.str();
}

std::string GCodeExport::floatconvertstring(const float ftNumber, const int nStrLen)
{
    int intPart = static_cast<int>(ftNumber);
    float fracPart = ftNumber - intPart;

    int nIntLen = 1;
    if (intPart > 0) {
        nIntLen += log10(intPart);
    }
    std::string int_str = std::to_string(intPart);

    if (nIntLen >= nStrLen){
        return int_str.substr(0, nStrLen);
    }
    else{
        std::string ret_str = int_str + ".";
        int nTempLen = ret_str.length();
        std::string float_str = std::to_string(fracPart);
        std::string sub_float_str = float_str.substr(2);
        int sub_float_str_len = sub_float_str.length();
        ret_str += sub_float_str;

        if (ret_str.length() >= nStrLen)
        {
            std::string tempstr = ret_str.substr(0, nStrLen);
            return ret_str.substr(0, nStrLen);
        }
        else
        {
            int restlen = nStrLen - ret_str.length();
            for (int nlen = 0; nlen < restlen; ++nlen)
            {
                ret_str += "0";
            }
            return ret_str;
        }
    }
}
}//namespace cura

