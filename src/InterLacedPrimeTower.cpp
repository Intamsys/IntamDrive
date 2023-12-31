#include "InterlacedPrimeTower.h"

#include <limits>
#include <cmath>
#include "ExtruderTrain.h"
#include "sliceDataStorage.h"
#include "gcodeExport.h"
#include "LayerPlan.h"
#include "infill.h"
#include "PrintFeature.h"
#include "raft.h"

#include <iostream>
#include <fstream>

#define CIRCLE_RESOLUTION 32 //The number of vertices in each circle.

namespace cura 
{

InterlacedPrimeTower::InterlacedPrimeTower()
{}

void InterlacedPrimeTower::Init(const SliceDataStorage& storage)
{
	is_hollow = false;
	wipe_from_middle = false;

	enabled = storage.getSettingBoolean("prime_tower_enable")
		&& storage.getSettingAsCount("prime_tower_line_count") > 0
		&& storage.getSettingInMicrons("prime_tower_size") > 10;
	if (enabled)
	{
		generateGroundpoly(storage);
	}
}

void InterlacedPrimeTower::generateGroundpoly(const SliceDataStorage& storage)
{
    extruder_count = storage.meshgroup->getExtruderCount();

	int64_t tower_size = storage.getSettingInMicrons("prime_tower_size");
	bool circular_prime_tower = storage.getSettingBoolean("prime_tower_circular");
	const coord_t layer_height = storage.getSettingInMicrons("layer_height");
	const coord_t prime_tower_line_count = storage.getSettingAsCount("prime_tower_line_count");
	int64_t prime_tower_wall_thickness = 0;
	int max_line_width = 0;

	for (int extruder = 0; extruder < extruder_count; extruder++)
	{
		int line_width = storage.meshgroup->getExtruderTrain(extruder)->getSettingInMicrons("prime_tower_line_width");
		if (line_width > max_line_width)	max_line_width = line_width;
	}

	prime_tower_wall_thickness = prime_tower_line_count * max_line_width;

    if (prime_tower_wall_thickness * 2 < tower_size)
    {
        is_hollow = true;
    }

    PolygonRef p = ground_poly.newPoly();
    int tower_distance = 0; 
    int x = storage.getSettingInMicrons("prime_tower_position_x"); // storage.model_max.x
    int y = storage.getSettingInMicrons("prime_tower_position_y"); // storage.model_max.y
    if (circular_prime_tower)
    {
        double_t tower_radius = tower_size / 2;
        for (unsigned int i = 0; i < CIRCLE_RESOLUTION; i++)
        {
            const double angle = (double)i / CIRCLE_RESOLUTION * 2 * M_PI; //In radians.
            p.add(Point(x - tower_radius + tower_distance + cos(angle) * tower_radius, y + tower_radius + tower_distance + sin(angle) * tower_radius));
        }
    }
    else 
	{
        p.add(Point(x + tower_distance, y + tower_distance));
        p.add(Point(x + tower_distance, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance + tower_size));
        p.add(Point(x + tower_distance - tower_size, y + tower_distance));
    }
    middle = Point(x - tower_size / 2, y + tower_size / 2);

    post_wipe_point = Point(x + tower_distance - tower_size / 2, y + tower_distance + tower_size / 2);
}

void InterlacedPrimeTower::generatePaths(const SliceDataStorage& storage)
{
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    size_t used_extruder_count = 0;
    for (bool is_used : extruder_is_used)
    {
        used_extruder_count += is_used;
    }
    enabled &= used_extruder_count >= 2;
    if (enabled)
    {
        generatePaths_denseInfill(storage);
        generateWipeLocations(storage);
    }
}

void InterlacedPrimeTower::generatePaths_denseInfill(const SliceDataStorage& storage)
{   
    coord_t current_line_count = 0;
    const coord_t prime_tower_line_count = storage.getSettingAsCount("prime_tower_line_count");
    const coord_t line_width = storage.getSettingInMicrons("prime_tower_line_width");

    for (int extruder = 0; extruder < extruder_count; extruder++)
    {
        pattern_per_extruder.emplace_back();
        ExtrusionMoves& pattern = pattern_per_extruder.back();
        int wall_idx = 0;
        current_line_count = 0;
        for (; current_line_count < prime_tower_line_count; wall_idx++)
        {
            //Create a new polygon with an offset from the outer polygon.
            Polygons polygons = ground_poly.offset(- wall_idx * line_width - line_width / 2);

            pattern.polygons.add(polygons);
            current_line_count++;
            if (polygons.empty()) //Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }

        int line_width_layer0 = line_width;
        if (storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
        {
            line_width_layer0 *= storage.getSettingAsRatio("initial_layer_line_width_factor");
        }
        pattern_per_extruder_layer0.emplace_back();
        ExtrusionMoves& pattern_layer0 = pattern_per_extruder_layer0.back();
        int wall_layer0_idx = 0;
        current_line_count = 0;
        for (; current_line_count < prime_tower_line_count; wall_layer0_idx++)
        {
            Polygons polygons = ground_poly.offset(-wall_layer0_idx * line_width_layer0 - line_width_layer0 / 2);
            pattern_layer0.polygons.add(polygons);
            current_line_count++;
            if (polygons.empty()) //Don't continue. We won't ever reach the required volume because it doesn't fit.
            {
                break;
            }
        }
    }
}

void InterlacedPrimeTower::addToGcode(const SliceDataStorage& storage, LayerPlan& gcode_layer, const GCodeExport& gcode, const int prev_extruder, const int new_extruder) const
{
    if (!enabled){
        return;
    }

	if (gcode_layer.getLayerNr() < 0){
		return;
	}
    
    if (gcode_layer.getPrimeTowerIsPlanned())
    { // don't print the prime tower if it has been printed already
        return;
    }

    if (storage.getSettingAsPrimeTowerHeightType("prime_tower_height_type") == PrimeTowerHeightType::NORMAL && gcode_layer.getLayerNr() > storage.max_print_height_second_to_last_extruder + 1)
    {
        return;
    }

    bool pre_wipe = storage.meshgroup->getExtruderTrain(new_extruder)->getSettingBoolean("dual_pre_wipe");
    bool post_wipe = storage.meshgroup->getExtruderTrain(prev_extruder)->getSettingBoolean("prime_tower_wipe_enabled");

    if (prev_extruder == new_extruder)
    {
        pre_wipe = false;
        post_wipe = false;
    }
    // pre-wipe:
    if (pre_wipe)
    {
        preWipeAndPurge(storage, gcode_layer, new_extruder);
    }

    addToGcode_denseInfill(storage, gcode_layer, new_extruder);

    // post-wipe:
    if (post_wipe)
    { //Make sure we wipe the old extruder on the prime tower.
        gcode_layer.addTravel(post_wipe_point - gcode.getExtruderOffset(prev_extruder) + gcode.getExtruderOffset(new_extruder));
    }
    gcode_layer.setPrimeTowerIsPlanned();
}

void InterlacedPrimeTower::addToGcode_denseInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr) const
{
    const ExtrusionMoves& pattern = (gcode_layer.getLayerNr() == 0)
        ? pattern_per_extruder_layer0[extruder_nr]
        : pattern_per_extruder[extruder_nr]; // +2) %2 to handle negative layer numbers

    const GCodePathConfig& config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];

    gcode_layer.addPolygonsByOptimizer(pattern.polygons, config);
    gcode_layer.addLinesByOptimizer(pattern.lines, config, SpaceFillType::Lines);
}

Point InterlacedPrimeTower::getLocationBeforePrimeTower(const SliceDataStorage& storage) const
{
    Point ret(0, 0);
    int absolute_starting_points = 0;
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(0);
        if (train.getSettingBoolean("machine_extruder_start_pos_abs"))
        {
            ret += Point(train.getSettingInMicrons("machine_extruder_start_pos_x"), train.getSettingInMicrons("machine_extruder_start_pos_y"));
            absolute_starting_points++;
        }
    }
    if (absolute_starting_points > 0)
    { // take the average over all absolute starting positions
		ret = ret * (1.0f / absolute_starting_points);
    }
    else
    { // use the middle of the bed
        if (!storage.getSettingBoolean("machine_center_is_zero"))
        {
            ret = Point(storage.getSettingInMicrons("machine_width"), storage.getSettingInMicrons("machine_depth")) / 2;
        }
        // otherwise keep (0, 0)
    }
    return ret;
}

void InterlacedPrimeTower::generateWipeLocations(const SliceDataStorage& storage)
{
    wipe_from_middle = is_hollow;
    // only wipe from the middle of the prime tower if we have a z hop already on the first move after the layer switch
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        const ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
        wipe_from_middle &= train.getSettingBoolean("retraction_hop_enabled") 
                        && (!train.getSettingBoolean("retraction_hop_only_when_collides") || train.getSettingBoolean("retraction_hop_after_extruder_switch"));
    }

    PolygonsPointIndex segment_start; // from where to start the sequence of wipe points
    PolygonsPointIndex segment_end; // where to end the sequence of wipe points

    if (wipe_from_middle)
    {
        // take the same start as end point so that the whole poly os covered.
        // find the inner polygon.
        segment_start = segment_end = PolygonUtils::findNearestVert(middle, ground_poly);
    }
    else
    {
        // take the closer corner of the wipe tower and generate wipe locations on that side only:
        //
        //     |
        //     |
        //     +-----
        //  .
        //  ^ nozzle switch location
        Point from = getLocationBeforePrimeTower(storage);

        // find the single line segment closest to [from] pointing most toward [from]
        PolygonsPointIndex closest_vert = PolygonUtils::findNearestVert(from, ground_poly);
        PolygonsPointIndex prev = closest_vert.prev();
        PolygonsPointIndex next = closest_vert.next();
        int64_t prev_dot_score = dot(from - closest_vert.p(), turn90CCW(prev.p() - closest_vert.p()));
        int64_t next_dot_score = dot(from - closest_vert.p(), turn90CCW(closest_vert.p() - next.p()));
        if (prev_dot_score > next_dot_score)
        {
            segment_start = prev;
            segment_end = closest_vert;
        }
        else
        {
            segment_start = closest_vert;
            segment_end = next;
        }
    }

    PolygonUtils::spreadDots(segment_start, segment_end, number_of_pre_wipe_locations, pre_wipe_locations);
}

void InterlacedPrimeTower::preWipeAndPurge(const SliceDataStorage& storage, LayerPlan& gcode_layer, const int extruder_nr) const
{
    int current_pre_wipe_location_idx = (pre_wipe_location_skip * gcode_layer.getLayerNr()) % number_of_pre_wipe_locations;
    const ClosestPolygonPoint wipe_location = pre_wipe_locations[current_pre_wipe_location_idx];

    const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    const int inward_dist = train->getSettingInMicrons("machine_nozzle_size") * 3 / 2 ;
    const int start_dist = train->getSettingInMicrons("machine_nozzle_size") * 2;
    const Point prime_end = PolygonUtils::moveInsideDiagonally(wipe_location, inward_dist);
    const Point outward_dir = wipe_location.location - prime_end;
    const Point prime_start = wipe_location.location + normal(outward_dir, start_dist);

    const double purge_volume = std::max(0.0, train->getSettingInCubicMillimeters("prime_tower_purge_volume")); // Volume to be primed
    if (wipe_from_middle)
    {
        // for hollow wipe tower:
        // start from above
        // go to wipe start
        // go to the Z height of the previous/current layer
        // wipe
        // go to normal layer height (automatically on the next extrusion move)...
        GCodePath& toward_middle = gcode_layer.addTravel(middle);
        toward_middle.perform_z_hop = true;
        gcode_layer.forceNewPathStart();

        if (purge_volume > 0)
        {
            addPurgeMove(gcode_layer, extruder_nr, train, middle, prime_start, purge_volume);
        }
        else
        {
            // Normal move behavior to wipe start location.
            GCodePath& toward_wipe_start = gcode_layer.addTravel_simple(prime_start);
            toward_wipe_start.perform_z_hop = false;
            toward_wipe_start.retract = true;
        }
    }
    else
    {
        if (purge_volume > 0)
        {
            // Find location to start purge (we're purging right outside of the tower)
            const Point purge_start = prime_start + normal(outward_dir, start_dist);
            gcode_layer.addTravel(purge_start);

            addPurgeMove(gcode_layer, extruder_nr, train, purge_start, prime_start, purge_volume);
        }
        gcode_layer.addTravel(prime_start);
    }

    float flow = 0.0001; // Force this path being interpreted as an extrusion path, so that no Z hop will occur (TODO: really separately handle travel and extrusion moves)
    gcode_layer.addExtrusionMove(prime_end, gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr], SpaceFillType::None, flow);
}

void InterlacedPrimeTower::subtractFromSupport(SliceDataStorage& storage)
{
    const Polygons outside_polygon = ground_poly.getOutsidePolygons();
    AABB outside_polygon_boundary_box(outside_polygon);
    for(size_t layer = 0; layer <= (size_t)storage.max_print_height_second_to_last_extruder + 1 && layer < storage.support.supportLayers.size(); layer++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer];
        // take the differences of the support infill parts and the prime tower area
        support_layer.excludeAreasFromSupportInfillAreas(outside_polygon, outside_polygon_boundary_box);
    }
}

void InterlacedPrimeTower::addPurgeMove(LayerPlan& gcode_layer, int extruder_nr, const ExtruderTrain *train, const Point& start_pos, const Point& end_pos, double purge_volume) const
{
    // Find out how much purging needs to be done.
    const GCodePathConfig& current_gcode_path_config = gcode_layer.configs_storage.prime_tower_config_per_extruder[extruder_nr];
    const coord_t purge_move_length = vSize(start_pos - end_pos);
    const unsigned int line_width = current_gcode_path_config.getLineWidth();
    const double layer_height_mm = (gcode_layer.getLayerNr() == 0) ? train->getSettingInMillimeters("layer_height_0") : train->getSettingInMillimeters("layer_height");
    const double normal_volume = INT2MM(INT2MM(purge_move_length * line_width)) * layer_height_mm; // Volume extruded on the "normal" move
    float purge_flow = purge_volume / normal_volume;

    const double purge_move_length_mm = INT2MM(purge_move_length);
    const double purge_move_time = purge_move_length_mm / current_gcode_path_config.getSpeed();
    const double purge_extrusion_speed_mm3_per_sec = purge_volume / purge_move_time;
    const double max_possible_extursion_speed_mm3_per_sec = 3.0;

    const double speed = current_gcode_path_config.getSpeed();
    double speed_factor = 1.0;

    if (purge_extrusion_speed_mm3_per_sec > max_possible_extursion_speed_mm3_per_sec)
    {
        // compensate the travel speed for the large extrusion amount
        const double min_time_needed_for_extrusion = purge_volume / max_possible_extursion_speed_mm3_per_sec;
        const double compensated_speed = purge_move_length_mm / min_time_needed_for_extrusion;
        speed_factor = compensated_speed / speed;
    }

    // As we need a plan, which can't have a stationary extrusion, we use an extrusion move to prime.
    // This has the added benefit that it will evenly spread the primed material inside the tower.
    gcode_layer.addExtrusionMove(end_pos, current_gcode_path_config, SpaceFillType::None, purge_flow, 1.0f, false, speed_factor);
}


}//namespace cura
