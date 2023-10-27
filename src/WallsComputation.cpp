//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "WallsComputation.h"
#include "utils/polygonUtils.h"
#include "ExtruderTrain.h"
#include "settings/types/Ratio.h"
#include "WallToolPaths.h"

namespace cura {

WallsComputation::WallsComputation(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const LayerIndex layer_nr)
: layer_nr(layer_nr)
{
}
void WallsComputation::generateWalls(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayer* layer)
{
	for (SliceLayerPart& part : layer->parts)
	{
		generateWalls(storage, mesh, &part);
	}

	//Remove the parts which did not generate a wall. As these parts are too small to print,
	// and later code can now assume that there is always minimal 1 wall line.
	if (mesh.getSettingAsCount("wall_line_count") >= 1 && !mesh.getSettingBoolean("fill_outline_gaps"))
	{
		for (size_t part_idx = 0; part_idx < layer->parts.size(); part_idx++)
		{
			if (layer->parts[part_idx].wall_toolpaths.empty() && layer->parts[part_idx].spiral_wall.empty())
			{
				if (part_idx != layer->parts.size() - 1)
				{ // move existing part into part to be deleted
					layer->parts[part_idx] = std::move(layer->parts.back());
				}
				layer->parts.pop_back(); // always remove last element from array (is more efficient)
				part_idx -= 1; // check the part we just moved here
			}
		}
	}
}

void WallsComputation::generateWalls(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayerPart* part)
{
	size_t wall_count = mesh.getSettingAsCount("wall_line_count");

	if (layer_nr == 0)
	{
		wall_count = mesh.getSettingAsCount("first_layer_wall_count");
	}

	if (wall_count == 0) // Early out if no walls are to be generated
	{
		part->print_outline = part->outline;
		part->inner_area = part->outline;
		return;
	}
	const bool spiralize = mesh.getSettingBoolean("magic_spiralize");
	const size_t alternate = ((layer_nr % 2) + 2) % 2;

	if (spiralize && layer_nr < LayerIndex(mesh.getSettingAsCount("bottom_layers")) && alternate == 1) //Add extra insets every 2 layers when spiralizing. This makes bottoms of cups watertight.
	{
		wall_count += 5;
	}
	if (mesh.getSettingBoolean("alternate_extra_perimeter"))
	{
		wall_count += alternate;
	}

	const bool first_layer = layer_nr == 0;

	const size_t wall_0_extruder_nr = mesh.getSettingAsExtruderNr("wall_0_extruder_nr");

	const double line_width_0_factor = first_layer ? storage.meshgroup->getExtruderTrain(wall_0_extruder_nr)->getSettingAsRatio("initial_layer_line_width_factor") : 1.0;
	const coord_t line_width_0 = mesh.getSettingInMicrons("wall_line_width_0") * line_width_0_factor;
	const coord_t wall_0_inset = mesh.getSettingInMicrons("wall_0_inset");

	const size_t wall_x_extruder_nr = mesh.getSettingAsExtruderNr("wall_x_extruder_nr");
	const double line_width_x_factor = first_layer ? storage.meshgroup->getExtruderTrain(wall_x_extruder_nr)->getSettingAsRatio("initial_layer_line_width_factor") : 1.0;
	const coord_t line_width_x = mesh.getSettingInMicrons("wall_line_width_x") * line_width_x_factor;

	// When spiralizing, generate the spiral insets using simple offsets instead of generating toolpaths

	if (spiralize)
	{
		const bool recompute_outline_based_on_outer_wall = mesh.getSettingBoolean("support_enable") && !mesh.getSettingBoolean("fill_outline_gaps");

		generateSpiralInsets(storage, mesh, part, line_width_0, wall_0_inset, recompute_outline_based_on_outer_wall);
		if (layer_nr <= static_cast<LayerIndex>(mesh.getSettingAsCount("bottom_layers")))
		{
			WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, mesh);
			part->wall_toolpaths = wall_tool_paths.getToolPaths(mesh);
			part->inner_area = wall_tool_paths.getInnerContour(mesh);
		}
	}
	else
	{
		WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_x, wall_count, wall_0_inset, mesh);
		part->wall_toolpaths = wall_tool_paths.getToolPaths(mesh);
		part->inner_area = wall_tool_paths.getInnerContour(mesh);
	}
	part->print_outline = part->outline;
}

void WallsComputation::generateSpiralInsets(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayerPart* part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall)
{
	part->spiral_wall = part->outline.offset(-line_width_0 / 2 - wall_0_inset);
	//Optimize the wall. This prevents buffer underruns in the printer firmware, and reduces processing time in CuraEngine.
	const ExtruderTrain& train_wall = *storage.meshgroup->getExtruderTrain(mesh.getSettingAsExtruderNr("wall_0_extruder_nr"));
	const coord_t maximum_resolution = train_wall.getSettingInMicrons("meshfix_maximum_resolution");
	const coord_t maximum_deviation = train_wall.getSettingInMicrons("meshfix_maximum_deviation");
	part->spiral_wall.simplify(maximum_resolution, maximum_deviation);
	part->spiral_wall.removeDegenerateVerts();
	if (recompute_outline_based_on_outer_wall)
	{
		part->print_outline = part->spiral_wall.offset(line_width_0 / 2, ClipperLib::jtSquare);
	}
	else
	{
		part->print_outline = part->outline;
	}
}

}//namespace cura
