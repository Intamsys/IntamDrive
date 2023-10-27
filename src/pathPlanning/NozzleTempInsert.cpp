//Copyright (C) 2016 Ultimaker
//Released under terms of the AGPLv3 License

#include "NozzleTempInsert.h"

namespace cura
{

NozzleTempInsert::NozzleTempInsert(unsigned int path_idx, int extruder, double temperature, bool wait, bool enable_preheat, double time_after_path_start)
: path_idx(path_idx)
, extruder(extruder)
, temperature(temperature)
, wait(wait)
, enable_preheat(enable_preheat)
, time_after_path_start(time_after_path_start)
{
    assert(temperature != 0 && temperature != -1 && "Temperature command must be set!");
}

void NozzleTempInsert::write(GCodeExport& gcode)
{
	//if(enable_preheat)
	gcode.writeTemperatureCommand(extruder, temperature, wait);
}

}