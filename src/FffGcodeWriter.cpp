//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <list>
#include <limits> // numeric_limits

#include "utils/math.h"
#include "FffGcodeWriter.h"
#include "FffProcessor.h"
#include "progress/Progress.h"
#include "utils/orderOptimizer.h"
#include "GcodeLayerThreader.h"
#include "infill/SpaghettiInfillPathGenerator.h"
#include "InsetOrderOptimizer.h"
#include "settings/types/LayerIndex.h" //For layer index settings.

#define OMP_MAX_ACTIVE_LAYERS_PROCESSED 50 // TODO: hardcoded-value for the max number of layers being in the pipeline while writing away and destroying layers in a multi-threaded context

namespace cura
{

FffGcodeWriter::FffGcodeWriter(SettingsBase* settings_)
: SettingsMessenger(settings_)
, max_object_height(0)
, layer_plan_buffer(this, gcode)
{
    for (unsigned int extruder_nr = 0; extruder_nr < MAX_EXTRUDERS; extruder_nr++)
    { // initialize all as max layer_nr, so that they get updated to the lowest layer on which they are used.
        extruder_prime_layer_nr[extruder_nr] = std::numeric_limits<int>::max();
    }
}

void FffGcodeWriter::writeGCode(SliceDataStorage& storage, TimeKeeper& time_keeper)
{
 	const size_t start_extruder_nr = getStartExtruder(storage);
	gcode.SetStartingExtruder(start_extruder_nr);
	CommandSocket::setExtruderForSend(start_extruder_nr);

    gcode.preSetup(storage.meshgroup);
	gcode.setPrimeTowerHeight(storage);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        gcode.resetTotalPrintTimeAndFilament();
        gcode.setInitialTemps(*storage.meshgroup, getStartExtruder(storage));
    }

    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->beginGCode();
    }

    setConfigFanSpeedLayerTime(storage);

    setConfigCoasting(storage);

    setConfigRetraction(storage);

	gcode.setExtruderRetractionConfig(storage.retraction_config_per_extruder);

    layer_plan_buffer.setPreheatConfig(*storage.meshgroup);
    
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    {
        unsigned int start_extruder_nr = getStartExtruder(storage);
        processStartingCode(storage, start_extruder_nr);
    }
    else
    {
        processNextMeshGroupCode(storage);
    }

    size_t total_layers = 0;
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.isPrinted()) //No need to process higher layers if the non-printed meshes are higher than the normal meshes.
        {
            total_layers = std::max(total_layers, mesh.layers.size());
        }
        setInfillAndSkinAngles(mesh);
    }

    setSupportAngles(storage);
    
    gcode.writeLayerCountComment(total_layers);

    { // calculate the mesh order for each extruder
        int extruder_count = storage.meshgroup->getExtruderCount();
        mesh_order_per_extruder.reserve(extruder_count);
        for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            mesh_order_per_extruder.push_back(calculateMeshOrder(storage, extruder_nr));
        }
    }
    calculateExtruderOrderPerLayer(storage);

    if (getSettingBoolean("magic_spiralize"))
    {
        findLayerSeamsForSpiralize(storage, total_layers);
    }

    int process_layer_starting_layer_nr = 0;
    bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;
    if (has_raft)
    {
        processRaft(storage);
        // process filler layers to fill the airgap with helper object (support etc) so that they stick better to the raft.
        // only process the filler layers if there is anything to print in them.
        for (bool extruder_is_used_in_filler_layers : storage.getExtrudersUsed(-1))
        {
            if (extruder_is_used_in_filler_layers)
            {
                process_layer_starting_layer_nr = -Raft::getFillerLayerCount(storage);
                break;
            }
        }
    }

    const std::function<LayerPlan* (int)>& produce_item =
        [&storage, total_layers, this](int layer_nr)
        {
            LayerPlan& gcode_layer = processLayer(storage, layer_nr, total_layers);
            return &gcode_layer;
        };
    const std::function<void (LayerPlan*)>& consume_item =
        [this, total_layers](LayerPlan* gcode_layer)
        {
            Progress::messageProgress(Progress::Stage::EXPORT, std::max(0, gcode_layer->getLayerNr()) + 1, total_layers);
            layer_plan_buffer.handle(*gcode_layer, gcode);
        };
    const unsigned int max_task_count = OMP_MAX_ACTIVE_LAYERS_PROCESSED;
    GcodeLayerThreader<LayerPlan> threader(
        process_layer_starting_layer_nr
        , static_cast<int>(total_layers)
        , produce_item
        , consume_item
        , max_task_count
    );

    // process all layers, process buffer for preheating and minimal layer time etc, write layers to gcode:
    threader.run();

    layer_plan_buffer.flush();

    Progress::messageProgressStage(Progress::Stage::FINISH, &time_keeper);

    //Store the object height for when we are printing multiple objects, as we need to clear every one of them when moving to the next position.
    max_object_height = std::max(max_object_height, storage.model_max.z);

    constexpr bool force = true;
    gcode.writeRetraction(storage.retraction_config_per_extruder[gcode.getExtruderNr()], force); // retract after finishing each meshgroup
}

unsigned int FffGcodeWriter::findSpiralizedLayerSeamVertexIndex(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int layer_nr, const int last_layer_nr)
{
    const SliceLayer& layer = mesh.layers[layer_nr];

    // last_layer_nr will be < 0 until we have processed the first non-empty layer
    if (last_layer_nr < 0)
    {
        // If the user has specified a z-seam location, use the vertex closest to that location for the seam vertex
        // in the first layer that has a part with insets. This allows the user to alter the seam start location which
        // could be useful if the spiralization has a problem with a particular seam path.
        Point seam_pos(0, 0);
        if (mesh.getSettingAsZSeamType("z_seam_type") == EZSeamType::USER_SPECIFIED)
        {
            seam_pos = mesh.getZSeamHint();
        }
        return PolygonUtils::findClosest(seam_pos, layer.parts[0].insets[0][0]).point_idx;
    }
    else
    {
        // note that the code below doesn't assume that last_layer_nr is one less than layer_nr but the print is going
        // to come out pretty weird if that isn't true as it implies that there are empty layers

        ConstPolygonRef last_wall = (*storage.spiralize_wall_outlines[last_layer_nr])[0];
        ConstPolygonRef wall = layer.parts[0].insets[0][0];
        const int n_points = wall.size();
        Point last_wall_seam_vertex = last_wall[storage.spiralize_seam_vertex_indices[last_layer_nr]];

        // seam_vertex_idx is going to be the index of the seam vertex in the current wall polygon
        // initially we choose the vertex that is closest to the seam vertex in the last spiralized layer processed

        int seam_vertex_idx = PolygonUtils::findClosest(last_wall_seam_vertex, wall).point_idx;

        // now we check that the vertex following the seam vertex is to the left of the seam vertex in the last layer
        // and if it isn't, we move forward

        // get the inward normal of the last layer seam vertex
        Point last_wall_seam_vertex_inward_normal = PolygonUtils::getVertexInwardNormal(last_wall, storage.spiralize_seam_vertex_indices[last_layer_nr]);

        // create a vector from the normal so that we can then test the vertex following the candidate seam vertex to make sure it is on the correct side
        Point last_wall_seam_vertex_vector = last_wall_seam_vertex + last_wall_seam_vertex_inward_normal;

        // now test the vertex following the candidate seam vertex and if it lies to the left of the vector, it's good to use
        const int first_seam_vertex_idx = seam_vertex_idx;
        float a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);

        while (a <= 0 || a >= M_PI)
        {
            // the vertex was not on the left of the vector so move the seam vertex on
            seam_vertex_idx = (seam_vertex_idx + 1) % n_points;
            a = LinearAlg2D::getAngleLeft(last_wall_seam_vertex_vector, last_wall_seam_vertex, wall[(seam_vertex_idx + 1) % n_points]);

            if (seam_vertex_idx == first_seam_vertex_idx)
            {
                logWarning("WARNING: findLayerSeamsForSpiralize() failed to find a suitable seam vertex on layer %d\n", layer_nr);
                // this shouldn't happen very often - I have seen it occur when the seam moves into a very sharp corner
                break;
            }
        }
        return seam_vertex_idx;
    }
}

void FffGcodeWriter::findLayerSeamsForSpiralize(SliceDataStorage& storage, size_t total_layers)
{
    // The spiral has to continue on in an anti-clockwise direction from where the last layer finished, it can't jump backwards

    // we track the seam position for each layer and ensure that the seam position for next layer continues in the right direction

    storage.spiralize_wall_outlines.reserve(total_layers);
    storage.spiralize_seam_vertex_indices.reserve(total_layers);

    int last_layer_nr = -1; // layer number of the last non-empty layer processed (for any extruder or mesh)

    for (unsigned layer_nr = 0; layer_nr < total_layers; ++layer_nr)
    {
        bool done_this_layer = false;

        // default is no information available
        storage.spiralize_wall_outlines[layer_nr] = nullptr;
        storage.spiralize_seam_vertex_indices[layer_nr] = 0;

        // iterate through extruders until we find a mesh that has a part with insets
        const std::vector<unsigned int>& extruder_order = extruder_order_per_layer[layer_nr];
        for (unsigned int extruder_idx = 0; !done_this_layer && extruder_idx < extruder_order.size(); ++extruder_idx)
        {
            const unsigned int extruder_nr = extruder_order[extruder_idx];
            // iterate through this extruder's meshes until we find a part with insets
            const std::vector<unsigned int>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (unsigned int mesh_idx : mesh_order)
            {
                SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                // if this mesh has layer data for this layer process it
                if (!done_this_layer && mesh.layers.size() > layer_nr)
                {
                    SliceLayer& layer = mesh.layers[layer_nr];
                    // if the first part in the layer (if any) has insets, process it
                    if (layer.parts.size() != 0 && layer.parts[0].insets.size() != 0)
                    {
                        // save the seam vertex index for this layer as we need it to determine the seam vertex index for the next layer
                        storage.spiralize_seam_vertex_indices[layer_nr] = findSpiralizedLayerSeamVertexIndex(storage, mesh, layer_nr, last_layer_nr);
                        // save the wall outline for this layer so it can be used in the spiralize interpolation calculation
                        storage.spiralize_wall_outlines[layer_nr] = &layer.parts[0].insets[0];
                        last_layer_nr = layer_nr;
                        // ignore any further meshes/extruders for this layer
                        done_this_layer = true;
                    }
                }
            }
        }
    }
}

void FffGcodeWriter::setConfigFanSpeedLayerTime(SliceDataStorage& storage)
{
    for (int extr = 0; extr < storage.meshgroup->getExtruderCount(); extr++)
    {
        fan_speed_layer_time_settings_per_extruder.emplace_back();
        FanSpeedLayerTimeSettings& fan_speed_layer_time_settings = fan_speed_layer_time_settings_per_extruder.back();
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extr);
        fan_speed_layer_time_settings.cool_min_layer_time = train->getSettingInSeconds("cool_min_layer_time");
        fan_speed_layer_time_settings.cool_min_layer_time_fan_speed_max = train->getSettingInSeconds("cool_min_layer_time_fan_speed_max");
        fan_speed_layer_time_settings.cool_fan_speed_0 = train->getSettingInPercentage("cool_fan_speed_0");
        fan_speed_layer_time_settings.cool_fan_speed_min = train->getSettingInPercentage("cool_fan_speed_min");
        fan_speed_layer_time_settings.cool_fan_speed_max = train->getSettingInPercentage("cool_fan_speed_max");
        fan_speed_layer_time_settings.cool_min_speed = train->getSettingInMillimetersPerSecond("cool_min_speed");
        fan_speed_layer_time_settings.cool_fan_full_layer = train->getSettingAsLayerNumber("cool_fan_full_layer");
		fan_speed_layer_time_settings.enable_auto_control = train->getSettingBoolean("cool_fan_auto_control_enabled");
        if (!train->getSettingBoolean("cool_fan_enabled"))
        {	
            fan_speed_layer_time_settings.cool_fan_speed_0 = 0;
            fan_speed_layer_time_settings.cool_fan_speed_min = 0;
            fan_speed_layer_time_settings.cool_fan_speed_max = 0;
        }
    }
}

void FffGcodeWriter::setConfigCoasting(SliceDataStorage& storage) 
{
    for (int extr = 0; extr < storage.meshgroup->getExtruderCount(); extr++)
    {
        storage.coasting_config.emplace_back();
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extr);
        CoastingConfig& coasting_config = storage.coasting_config.back();
        coasting_config.coasting_enable = train->getSettingBoolean("coasting_enable"); 
        coasting_config.coasting_volume = std::max(0.0, train->getSettingInCubicMillimeters("coasting_volume"));
        coasting_config.coasting_min_volume = std::max(0.0, train->getSettingInCubicMillimeters("coasting_min_volume"));
        coasting_config.coasting_speed = train->getSettingInPercentage("coasting_speed") / 100.0; 
    }
}

void FffGcodeWriter::setConfigRetraction(SliceDataStorage& storage) 
{
    int extruder_count = storage.meshgroup->getExtruderCount();
    for (int extruder = 0; extruder < extruder_count; extruder++)
    {
        ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder);
        RetractionConfig& retraction_config = storage.retraction_config_per_extruder[extruder];
        retraction_config.distance = (train->getSettingBoolean("retraction_enable"))? train->getSettingInMillimeters("retraction_amount") : 0;
        retraction_config.prime_volume = train->getSettingInCubicMillimeters("retraction_extra_prime_amount");
        retraction_config.speed = train->getSettingInMillimetersPerSecond("retraction_retract_speed");
        retraction_config.primeSpeed = train->getSettingInMillimetersPerSecond("retraction_prime_speed");
        retraction_config.zHop = train->getSettingInMicrons("retraction_hop");
        retraction_config.retraction_min_travel_distance = train->getSettingInMicrons("retraction_min_travel");
        retraction_config.retraction_extrusion_window = train->getSettingInMillimeters("retraction_extrusion_window");
        retraction_config.retraction_count_max = train->getSettingAsCount("retraction_count_max");

        RetractionConfig& switch_retraction_config = storage.extruder_switch_retraction_config_per_extruder[extruder];
        switch_retraction_config.distance = train->getSettingInMillimeters("switch_extruder_retraction_amount"); 
        switch_retraction_config.prime_volume = 0.0;
        switch_retraction_config.speed = train->getSettingInMillimetersPerSecond("switch_extruder_retraction_speed");
        switch_retraction_config.primeSpeed = train->getSettingInMillimetersPerSecond("switch_extruder_prime_speed");
        switch_retraction_config.zHop = retraction_config.zHop; // not used, because the last_retraction_config is used to govern how how high to zHop
        switch_retraction_config.retraction_min_travel_distance = 0; // no limitation on travel distance for an extruder switch retract
        switch_retraction_config.retraction_extrusion_window = 99999.9; // so that extruder switch retractions won't affect the retraction buffer (extruded_volume_at_previous_n_retractions)
        switch_retraction_config.retraction_count_max = 9999999; // extruder switch retraction is never limited
    }
}

unsigned int FffGcodeWriter::getStartExtruder(const SliceDataStorage& storage)
{
    const EPlatformAdhesion adhesion_type = storage.meshgroup->getSettingAsPlatformAdhesion("adhesion_type");
    const int skirt_brim_extruder_nr = storage.meshgroup->getSettingAsIndex("skirt_brim_extruder_nr");
    const ExtruderTrain& skirt_brim_extruder = *storage.meshgroup->getExtruderTrain(skirt_brim_extruder_nr);
    int start_extruder_nr = 0;
    if (adhesion_type == EPlatformAdhesion::SKIRT)
    {
        if (skirt_brim_extruder.getSettingAsCount("skirt_line_count") > 0 || skirt_brim_extruder.getSettingInMicrons("skirt_brim_minimal_length") > 0)
        {
            start_extruder_nr = skirt_brim_extruder_nr;
        }
    }
    else if(adhesion_type == EPlatformAdhesion::BRIM || getSettingBoolean("prime_tower_brim_enable"))
    {
        if (skirt_brim_extruder.getSettingAsCount("brim_line_count") > 0 || skirt_brim_extruder.getSettingInMicrons("skirt_brim_minimal_length") > 0)
        {
            start_extruder_nr = skirt_brim_extruder_nr;
        }
    }
    else if (adhesion_type == EPlatformAdhesion::RAFT)
    {
        start_extruder_nr = storage.meshgroup->getSettingAsIndex("raft_base_extruder_nr");
    }
    else
    {
        std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
        for (unsigned int extruder_nr = 0; extruder_nr < extruder_is_used.size(); extruder_nr++)
        {
            start_extruder_nr = extruder_nr;
            if (extruder_is_used[extruder_nr])
            {
                break;
            }
        }
    }
    assert(start_extruder_nr >= 0 && start_extruder_nr < storage.meshgroup->getExtruderCount() && "start_extruder_nr must be a valid extruder");
    return start_extruder_nr;
}

void FffGcodeWriter::setInfillAndSkinAngles(SliceMeshStorage& mesh)
{
    if (mesh.infill_angles.size() == 0)
    {
        mesh.infill_angles = mesh.getSettingAsIntegerList("infill_angles");
        if (mesh.infill_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            EFillMethod infill_pattern = mesh.getSettingAsFillMethod("infill_pattern");
            if (infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                mesh.infill_angles.push_back(22); // put most infill lines in between 45 and 0 degrees
            }
            else
            {
                mesh.infill_angles.push_back(45); // generally all infill patterns use 45 degrees
                if (infill_pattern == EFillMethod::LINES || infill_pattern == EFillMethod::ZIG_ZAG)
                {
                    // lines and zig zag patterns default to also using 135 degrees
                    mesh.infill_angles.push_back(135);
                }
            }
        }
    }

    if (mesh.roofing_angles.size() == 0)
    {
        mesh.roofing_angles = mesh.getSettingAsIntegerList("roofing_angles");
        if (mesh.roofing_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.roofing_angles.push_back(45);
            mesh.roofing_angles.push_back(135);
        }
    }

    if (mesh.flooring_angles.size() == 0)
    {
        mesh.flooring_angles = mesh.getSettingAsIntegerList("flooring_angles");
        if (mesh.flooring_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.flooring_angles.push_back(45);
            mesh.flooring_angles.push_back(135);
        }
    }

    if (mesh.skin_angles.size() == 0)
    {
        mesh.skin_angles = mesh.getSettingAsIntegerList("skin_angles");
        if (mesh.skin_angles.size() == 0)
        {
            // user has not specified any infill angles so use defaults
            mesh.skin_angles.push_back(45);
            mesh.skin_angles.push_back(135);
        }
    }
}

void FffGcodeWriter::setSupportAngles(SliceDataStorage& storage)
{
    int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
    const ExtruderTrain& support_infill_extruder = *storage.meshgroup->getExtruderTrain(support_infill_extruder_nr);
    storage.support.support_infill_angles = support_infill_extruder.getSettingAsIntegerList("support_infill_angles");
    if (storage.support.support_infill_angles.empty())
    {
        storage.support.support_infill_angles.push_back(0);
    }

    int support_extruder_layer_0 = storage.getSettingAsIndex("support_extruder_nr_layer_0");
    const ExtruderTrain& support_extruder_nr_layer_0 = *storage.meshgroup->getExtruderTrain(support_extruder_layer_0);
    storage.support.support_infill_angles_layer_0 = support_extruder_nr_layer_0.getSettingAsIntegerList("support_infill_angles");
    if (storage.support.support_infill_angles_layer_0.empty())
    {
        storage.support.support_infill_angles_layer_0.push_back(0);
    }

    auto getInterfaceAngles = [&storage](const ExtruderTrain& extruder, const std::string& interface_angles_setting, const EFillMethod pattern, const std::string& interface_height_setting)
    {
        std::vector<int> angles = extruder.getSettingAsIntegerList(interface_angles_setting);
        if (angles.empty())
        {
            if (pattern == EFillMethod::CONCENTRIC)
            {
                angles.push_back(0); //Concentric has no rotation.
            }
            else if (pattern == EFillMethod::TRIANGLES)
            {
                angles.push_back(90); //Triangular support interface shouldn't alternate every layer.
            }
            else
            {
                for (const SliceMeshStorage& mesh : storage.meshes)
                {
                    if (mesh.getSettingInMicrons(interface_height_setting) >= 2 * mesh.getSettingInMicrons("layer_height"))
                    {
                        //Some roofs are quite thick.
                        //Alternate between the two kinds of diagonal: / and \ .
                        angles.push_back(45);
                        angles.push_back(135);
                    }
                }
                if (angles.empty())
                {
                    angles.push_back(90); //Perpendicular to support lines.
                }
            }
        }
        return angles;
    };

    int support_roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
    const ExtruderTrain& roof_extruder = *storage.meshgroup->getExtruderTrain(support_roof_extruder_nr);
    storage.support.support_roof_angles = getInterfaceAngles(roof_extruder, "support_roof_angles", roof_extruder.getSettingAsFillMethod("support_roof_pattern"), "support_roof_height");

    int support_bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
    const ExtruderTrain& bottom_extruder = *storage.meshgroup->getExtruderTrain(support_bottom_extruder_nr);
    storage.support.support_bottom_angles = getInterfaceAngles(bottom_extruder, "support_bottom_angles", bottom_extruder.getSettingAsFillMethod("support_bottom_pattern"), "support_bottom_height");
}

void FffGcodeWriter::processStartingCode(const SliceDataStorage& storage, const unsigned int start_extruder_nr)    
{
    std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    const unsigned num_extruders = storage.getSettingAsCount("machine_extruder_count");

    if (!CommandSocket::isInstantiated())
    {
        double print_time = gcode.getSumTotalPrintTimes();
        std::vector<double> filament_used;
        std::vector<std::string> material_names;
        std::vector<std::string> material_keys;
        std::vector<double>filament_weights;
        for (int extr_nr = 0; extr_nr < getSettingAsCount("machine_extruder_count"); extr_nr++)
        {
            filament_used.emplace_back(gcode.getTotalFilamentUsed(extr_nr));
            material_names.emplace_back(gcode.getMaterialNAME(extr_nr));
            material_keys.emplace_back(gcode.getMaterialKEY(extr_nr));
            filament_weights.emplace_back(gcode.getTotalFilamentWeight(extr_nr));
        }

        std::string prefix = gcode.getFileHeaderInfo(extruder_is_used, &print_time, filament_used, filament_weights, material_names, material_keys);
        gcode.writeCode(prefix.c_str());
    }
	
	double chamber_temp = -1;
	int extruder_count = storage.meshgroup->getExtruderCount();
    int outer_wall_index = storage.meshes[0].getSettingAsExtruderNr("wall_0_extruder_nr");
    if (outer_wall_index < 0 || outer_wall_index > extruder_count - 1)
        outer_wall_index = 0;

    chamber_temp = storage.meshgroup->getExtruderTrain(outer_wall_index)->getSettingInDegreeCelsius("chamber_temperature");
	if (chamber_temp >= 0)
	{
		if (gcode.getFlavor() != EGCodeFlavor::ULTIGCODE)
		{
			std::ostringstream tmp;
			tmp << "M141 S" << chamber_temp;
			gcode.writeLine(tmp.str().c_str());
		}
	}

	double bedTemperature = storage.meshgroup->getExtruderTrain(outer_wall_index)->getSettingInDegreeCelsius("material_bed_temperature");

	gcode.writeBedTemperatureCommand(bedTemperature,false);

    gcode.setBuildPlateAndChamberTemperture(bedTemperature, chamber_temp);

    if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    {
        gcode.writeCode("T0"); // required before any temperature setting commands
    }

    if (gcode.getFlavor() != EGCodeFlavor::ULTIGCODE && gcode.getFlavor() != EGCodeFlavor::GRIFFIN)
    {
        if (num_extruders > 1 || gcode.getFlavor() == EGCodeFlavor::REPRAP)
        {
            std::ostringstream tmp;
            tmp << "T" << start_extruder_nr;
            gcode.writeLine(tmp.str().c_str());
        }

        if (getSettingBoolean("material_bed_temp_prepend"))
        {
            if (getSettingBoolean("machine_heated_bed"))
            {
                double bed_temp_layer_0 = storage.meshgroup->getExtruderTrain(outer_wall_index)->getSettingInDegreeCelsius("material_bed_temperature_layer_0");

                if (bed_temp_layer_0 > 0)
                {
                    gcode.writeBedTemperatureCommand(bed_temp_layer_0, getSettingBoolean("material_bed_temp_wait"));
                }
            }
        }

        if (getSettingBoolean("material_print_temp_prepend"))
        {
            for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
            {
                if (extruder_is_used[extruder_nr])
                {
                    ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
                    double extruder_temp;
                    if (extruder_nr == start_extruder_nr)
                    {
                        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
                        extruder_temp = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
                    }
                    else
                    {
                        extruder_temp = train.getSettingInDegreeCelsius("material_standby_temperature");
                    }
                    gcode.writeTemperatureCommand(extruder_nr, extruder_temp, false, true);
                }
            }
            if (getSettingBoolean("material_print_temp_wait"))
            {
                for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
                {
                    if (extruder_is_used[extruder_nr])
                    {
                        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
                        double extruder_temp;
                        if (extruder_nr == start_extruder_nr)
                        {
                            double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
                            extruder_temp = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
                        }
                        else
                        {
                            extruder_temp = train.getSettingInDegreeCelsius("material_standby_temperature");
                        }
                        gcode.writeTemperatureCommand(extruder_nr, extruder_temp, true, true);
                    }
                }
            }
        }
    }

    gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
    gcode.writeCode(getSettingString("machine_start_gcode").c_str());

    if (gcode.getFlavor() == EGCodeFlavor::BFB)
    {
        gcode.writeComment("enable auto-retraction");
        std::ostringstream tmp;
        tmp << "M227 S" << (getSettingInMicrons("retraction_amount") * 2560 / 1000) << " P" << (getSettingInMicrons("retraction_amount") * 2560 / 1000);
        gcode.writeLine(tmp.str().c_str());
    }
    else if (gcode.getFlavor() == EGCodeFlavor::GRIFFIN)
    { // initialize extruder trains
        CommandSocket::setSendCurrentPosition(gcode.getPositionXY());
        gcode.startExtruder(start_extruder_nr);
        ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(start_extruder_nr);
        constexpr bool wait = true;
        double print_temp_0 = train.getSettingInDegreeCelsius("material_print_temperature_layer_0");
        double print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.getSettingInDegreeCelsius("material_print_temperature");
        gcode.writeTemperatureCommand(start_extruder_nr, print_temp_here, wait);
        gcode.writePrimeTrain(train.getSettingInMillimetersPerSecond("speed_travel"));
        extruder_prime_layer_nr[start_extruder_nr] = std::numeric_limits<int>::min(); // set to most negative number so that layer processing never primes this extruder any more.
        const RetractionConfig& retraction_config = storage.retraction_config_per_extruder[start_extruder_nr];
        gcode.writeRetraction(retraction_config);
    }

	for (unsigned extruder_nr = 0; extruder_nr < num_extruders; extruder_nr++)
	{
		if(extruder_nr != start_extruder_nr)
		{
			ExtruderTrain& train = *storage.meshgroup->getExtruderTrain(extruder_nr);
			train.setSetting("prime_blob_enable", "false");
		}
	}

    if (getSettingBoolean("relative_extrusion"))
    {
        gcode.writeExtrusionMode(true);
    }
    if (gcode.getFlavor() != EGCodeFlavor::GRIFFIN)
    {
        if (getSettingBoolean("retraction_enable"))
        {
            // ensure extruder is zeroed
            gcode.resetExtrusionValue();

            // retract before first travel move
            gcode.writeRetraction(storage.retraction_config_per_extruder[start_extruder_nr]);
        }
    }
}

void FffGcodeWriter::processNextMeshGroupCode(const SliceDataStorage& storage)
{
    gcode.writeFanCommand(0, gcode.getExtruderNr());
    gcode.setZ(max_object_height + 5000);

    if (storage.getSettingBoolean("machine_heated_bed") && storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0") != 0)
    {
        constexpr bool wait = true;
        double bed_temp = storage.getSettingInDegreeCelsius("material_bed_temperature_layer_0");
        double bed_temp_nr = storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInDegreeCelsius("material_bed_temperature_layer_0");
        gcode.writeBedTemperatureCommand(std::max(bed_temp, bed_temp_nr), wait);
    }

    CommandSocket::setSendCurrentPosition(gcode.getPositionXY());
    gcode.writeTravel(gcode.getPositionXY(), storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"));
    Point start_pos(storage.model_min.x, storage.model_min.y);
    gcode.writeTravel(start_pos, storage.meshgroup->getExtruderTrain(gcode.getExtruderNr())->getSettingInMillimetersPerSecond("speed_travel"));
}
    
void FffGcodeWriter::processRaft(const SliceDataStorage& storage)
{
    int raft_base_extruder_nr = getSettingAsIndex("raft_base_extruder_nr");
    int raft_interface_extruder_nr = getSettingAsIndex("raft_interface_extruder_nr");
    int raft_surface_extruder_nr = getSettingAsIndex("raft_surface_extruder_nr");
    int raft_skin_extruder_nr = getSettingAsIndex("raft_skin_extruder_nr");

    ExtruderTrain* raft_base_train = storage.meshgroup->getExtruderTrain(raft_base_extruder_nr);
    ExtruderTrain* raft_interface_train = storage.meshgroup->getExtruderTrain(raft_interface_extruder_nr);
    ExtruderTrain* raft_surface_train = storage.meshgroup->getExtruderTrain(raft_surface_extruder_nr);
    ExtruderTrain* raft_skin_train = storage.meshgroup->getExtruderTrain(raft_skin_extruder_nr);
    
    CombingMode combing_mode = storage.getSettingAsCombingMode("retraction_combing"); 

    int z = 0;
    const int initial_raft_layer_nr = -Raft::getTotalExtraLayers(storage);

    // some infill config for all lines infill generation below
    constexpr int offset_from_poly_outline = 0;
    constexpr double fill_overlap = 0; // raft line shouldn't be expanded - there is no boundary polygon printed
    constexpr int infill_multiplier = 1; // rafts use single lines
    constexpr int extra_infill_shift = 0;
    Polygons raft_polygons; // should remain empty, since we only have the lines pattern for the raft...
    std::optional<Point> last_planned_position = std::optional<Point>();

    const size_t num_interface_layers = storage.getSettingAsCount("raft_interface_layers");
    const size_t num_surface_layers = storage.getSettingAsCount("raft_surface_layers");

    unsigned int current_extruder_nr = raft_base_extruder_nr;

    { // raft base layer
        int layer_nr = initial_raft_layer_nr;
        int layer_height = raft_base_train->getSettingInMicrons("raft_base_thickness");
        z += layer_height;
        int64_t comb_offset = raft_base_train->getSettingInMicrons("raft_base_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_base = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_base)
        {
            double regular_fan_speed = raft_base_train->getSettingInPercentage("raft_base_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

		int nozzle_offset_z = raft_base_train->getSettingInMicrons("machine_nozzle_offset_z");
		z += nozzle_offset_z;

        LayerPlan& gcode_layer = *new LayerPlan(*this, storage, layer_nr, z, layer_height, raft_base_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_base, combing_mode, comb_offset, raft_base_train->getSettingBoolean("travel_avoid_other_parts"), raft_base_train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(raft_base_extruder_nr);

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons raftLines;
        double fill_angle = 0;
        constexpr bool zig_zaggify_infill = false;
        constexpr bool connect_polygons = true; // causes less jerks, so better adhesion
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool skip_stitching = false;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr bool apply_pockets_alternatingly = false;
        constexpr coord_t pocket_size = 0;
        const size_t wall_line_count = raft_base_train->getSettingAsCount("raft_base_wall_count");
        const coord_t max_resolution = raft_base_train->getSettingInMicrons("meshfix_maximum_resolution");
        const coord_t max_deviation = raft_base_train->getSettingInMicrons("meshfix_maximum_deviation");

        bool retract_before_outer_wall = storage.getSettingBoolean("travel_retract_before_outer_wall");
        int wipe_dist = storage.getSettingInMicrons("infill_wipe_dist");

        Infill infill_comp(
            EFillMethod::LINES, zig_zaggify_infill, connect_polygons, storage.raftOutline, gcode_layer.configs_storage.raft_base_config.getLineWidth(), raft_base_train->getSettingInMicrons("raft_base_line_spacing"), 
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            max_resolution, max_deviation,
            wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
            );

        std::vector<VariableWidthLines> raft_paths;
        infill_comp.generate(raft_paths, raft_polygons, raftLines, *raft_base_train);
        if (!raft_paths.empty())
        {
            const GCodePathConfig& config = gcode_layer.configs_storage.raft_base_config;
            const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
 
            InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, *raft_base_train, raft_base_extruder_nr,
                config, config, config, config,
                retract_before_outer_wall, wipe_dist, wipe_dist, raft_base_extruder_nr, raft_base_extruder_nr, z_seam_config, raft_paths);
             wall_orderer.addToLayer();
        }

        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_base_config, SpaceFillType::Lines);

        // When we use raft, we need to make sure that all used extruders for this print will get primed on the first raft layer,
        // and then switch back to the original extruder.
        std::vector<unsigned int> extruder_order = getUsedExtrudersOnLayerExcludingStartingExtruder(storage, raft_base_extruder_nr, layer_nr);
        for (unsigned int to_be_primed_extruder_nr : extruder_order)
        {
            setExtruder_addPrime(storage, gcode_layer, to_be_primed_extruder_nr);
            current_extruder_nr = to_be_primed_extruder_nr;
        }

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

	for (int raft_interface_layer = 1; raft_interface_layer <= raft_interface_train->getSettingAsCount("raft_interface_layers"); raft_interface_layer++)
    { // raft interface layer
        int layer_nr = initial_raft_layer_nr + raft_interface_layer;
        int layer_height = raft_interface_train->getSettingInMicrons("raft_interface_thickness");
        z += layer_height;
        int64_t comb_offset = raft_interface_train->getSettingInMicrons("raft_interface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_interface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_interface)
        {
            double regular_fan_speed = raft_interface_train->getSettingInPercentage("raft_interface_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        const std::vector<unsigned int>& extruder_order = (layer_nr < 0) ?
            extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr] : extruder_order_per_layer[layer_nr];

        LayerPlan& gcode_layer = *new LayerPlan(*this, storage, layer_nr, z, layer_height, current_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_interface, combing_mode, comb_offset, raft_interface_train->getSettingBoolean("travel_avoid_other_parts"), raft_interface_train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        gcode_layer.setExtruder(raft_interface_extruder_nr); // reset to extruder number, because we might have primed in the last layer
        current_extruder_nr = raft_interface_extruder_nr;

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_height);
        }

        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_interface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(); //Remove those micron-movements.
        //constexpr coord_t infill_outline_width = 0;
        Polygons raftLines;
        int offset_from_poly_outline = 0;

		double fill_angle = 0;
		if (raft_interface_train->getSettingAsCount("raft_surface_layers") == 0)
		{
			fill_angle = (raft_interface_layer % 2 == 0) ? 0 : 90;
		}
		else
		{
			fill_angle = (raft_interface_layer % 2 == 0) ? 135 : 45;
		}
        const coord_t infill_outline_width = gcode_layer.configs_storage.raft_interface_config.getLineWidth();

        constexpr bool zig_zaggify_infill = true;
        constexpr bool connect_polygons = true; // causes less jerks, so better adhesion
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connected_zigzags = false;
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr bool apply_pockets_alternatingly = false;
        constexpr coord_t pocket_size = 0;
        constexpr bool skip_stitching = false;
        const size_t wall_line_count = 0;
        const coord_t max_resolution = raft_interface_train->getSettingInMicrons("meshfix_maximum_resolution");
        const coord_t max_deviation = raft_interface_train->getSettingInMicrons("meshfix_maximum_deviation");

        Infill infill_comp(
            EFillMethod::ZIG_ZAG, zig_zaggify_infill, connect_polygons, raft_outline_path, gcode_layer.configs_storage.raft_interface_config.getLineWidth(), raft_interface_train->getSettingInMicrons("raft_interface_line_spacing"), fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            max_resolution, max_deviation,
            wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
        );

        std::vector<VariableWidthLines> raft_paths; //Should remain empty, since we have no walls.
        infill_comp.generate(raft_paths, raft_polygons, raftLines, *raft_interface_train);
        gcode_layer.addLinesByOptimizer(raftLines, gcode_layer.configs_storage.raft_interface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }
    
    int raft_surface_layer_height = raft_surface_train->getSettingInMicrons("raft_surface_thickness");

    for (int raft_surface_layer = 1; raft_surface_layer <= raft_surface_train->getSettingAsCount("raft_surface_layers"); raft_surface_layer++)
    { // raft surface layers
        const int layer_nr = initial_raft_layer_nr + 1 + raft_interface_train->getSettingAsCount("raft_interface_layers") + raft_surface_layer - 1; //
        z += raft_surface_layer_height;
        const int64_t comb_offset = raft_surface_train->getSettingInMicrons("raft_surface_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_surface = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_surface)
        {
            double regular_fan_speed = raft_surface_train->getSettingInPercentage("raft_surface_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed; // ignore initial layer fan speed stuff
        }

        const std::vector<unsigned int>& extruder_order =
            (layer_nr < 0) ?
            extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr] : extruder_order_per_layer[layer_nr];

        LayerPlan& gcode_layer = *new LayerPlan(*this, storage, layer_nr, z, raft_surface_layer_height, current_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_surface, combing_mode, comb_offset, raft_surface_train->getSettingBoolean("travel_avoid_other_parts"), raft_surface_train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        // make sure that we are using the correct extruder to print raft
        gcode_layer.setExtruder(raft_surface_extruder_nr);
        current_extruder_nr = raft_surface_extruder_nr;

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, raft_surface_layer_height);
        }

        const coord_t maximum_resolution = raft_surface_train->getSettingInMicrons("meshfix_maximum_resolution");
        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_surface_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(maximum_resolution); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        double fill_angle = 90 * raft_surface_layer;
        constexpr bool zig_zaggify_infill = true;
        constexpr bool connected_zigzags = false;
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connect_polygons = false; // midway connections between polygons can make the surface less smooth
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr bool apply_pockets_alternatingly = false;
        constexpr coord_t pocket_size = 0;
        constexpr bool skip_stitching = false;
        const size_t wall_line_count = 0;
        const coord_t surface_max_resolution = raft_surface_train->getSettingInMicrons("meshfix_maximum_resolution");
        const coord_t surface_max_deviation = raft_surface_train->getSettingInMicrons("meshfix_maximum_deviation");

        Infill infill_comp(
            EFillMethod::ZIG_ZAG, zig_zaggify_infill, connect_polygons, raft_outline_path, gcode_layer.configs_storage.raft_surface_config.getLineWidth(), raft_surface_train->getSettingInMicrons("raft_surface_line_spacing"),
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            surface_max_resolution, surface_max_deviation,
            wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
        );

        std::vector<VariableWidthLines> raft_paths; //Should remain empty, since we have no walls.
        infill_comp.generate(raft_paths, raft_polygons, raft_lines, *raft_surface_train);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_surface_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        layer_plan_buffer.handle(gcode_layer, gcode);
        last_planned_position = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }

    int raft_skin_layer_height = raft_skin_train->getSettingInMicrons("raft_skin_thickness");
    { // raft skin layer
        const int layer_nr = initial_raft_layer_nr + 1 + raft_interface_train->getSettingAsCount("raft_interface_layers") + raft_surface_train->getSettingAsCount("raft_surface_layers");
        int raft_skin_layer_height = raft_skin_train->getSettingInMicrons("raft_skin_thickness");
        z += raft_skin_layer_height;
        const int64_t comb_offset = raft_skin_train->getSettingInMicrons("raft_skin_line_spacing");

        std::vector<FanSpeedLayerTimeSettings> fan_speed_layer_time_settings_per_extruder_raft_skin = fan_speed_layer_time_settings_per_extruder; // copy so that we change only the local copy
        for (FanSpeedLayerTimeSettings& fan_speed_layer_time_settings : fan_speed_layer_time_settings_per_extruder_raft_skin)
        {
            double regular_fan_speed = raft_skin_train->getSettingInPercentage("raft_skin_fan_speed");
            fan_speed_layer_time_settings.cool_fan_speed_min = regular_fan_speed;
            fan_speed_layer_time_settings.cool_fan_speed_0 = regular_fan_speed;
        }

        const std::vector<unsigned int>& extruder_order =
            (layer_nr < 0) ?
            extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr] : extruder_order_per_layer[layer_nr];

        LayerPlan& gcode_layer = *new LayerPlan(*this, storage, layer_nr, z, raft_skin_layer_height, current_extruder_nr, fan_speed_layer_time_settings_per_extruder_raft_skin, combing_mode, comb_offset, raft_skin_train->getSettingBoolean("travel_avoid_other_parts"), raft_skin_train->getSettingInMicrons("travel_avoid_distance"));
        gcode_layer.setIsInside(true);

        // make sure that we are using the correct extruder to print raft
        gcode_layer.setExtruder(raft_skin_extruder_nr);
        current_extruder_nr = raft_skin_extruder_nr;

        if (CommandSocket::isInstantiated())
        {
            CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, raft_skin_layer_height);
        }

        const coord_t maximum_resolution = raft_skin_train->getSettingInMicrons("meshfix_maximum_resolution");
        Polygons raft_outline_path = storage.raftOutline.offset(-gcode_layer.configs_storage.raft_skin_config.getLineWidth() / 2); //Do this manually because of micron-movement created in corners when insetting a polygon that was offset with round joint type.
        raft_outline_path.simplify(maximum_resolution); //Remove those micron-movements.
        constexpr coord_t infill_outline_width = 0;
        Polygons raft_lines;
        int offset_from_poly_outline = 0;
        double fill_angle = 90 * (raft_surface_train->getSettingAsCount("raft_surface_layers") + 1);

        constexpr bool zig_zaggify_infill = true;
        constexpr bool connected_zigzags = false;
        const Point& infill_origin = Point();
        Polygons* perimeter_gaps = nullptr;
        constexpr bool connect_polygons = false; // midway connections between polygons can make the surface less smooth
        constexpr bool use_endpieces = true;
        constexpr bool skip_some_zags = false;
        constexpr int zag_skip_count = 0;
        constexpr bool apply_pockets_alternatingly = false;
        constexpr coord_t pocket_size = 0;
        constexpr bool skip_stitching = false;
        const size_t wall_line_count = 0;
        const coord_t surface_max_resolution = raft_skin_train->getSettingInMicrons("meshfix_maximum_resolution");
        const coord_t surface_max_deviation = raft_skin_train->getSettingInMicrons("meshfix_maximum_deviation");

        Infill infill_comp(
            EFillMethod::ZIG_ZAG, zig_zaggify_infill, connect_polygons, raft_outline_path, gcode_layer.configs_storage.raft_skin_config.getLineWidth(), raft_skin_train->getSettingInMicrons("raft_skin_line_spacing"),
            fill_overlap, infill_multiplier, fill_angle, z, extra_infill_shift,
            surface_max_resolution, surface_max_deviation,
            wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
        );

        std::vector<VariableWidthLines> raft_paths; //Should remain empty, since we have no walls.
        infill_comp.generate(raft_paths, raft_polygons, raft_lines, *raft_skin_train);
        gcode_layer.addLinesByOptimizer(raft_lines, gcode_layer.configs_storage.raft_skin_config, SpaceFillType::Lines, false, 0, 1.0, last_planned_position);

        layer_plan_buffer.handle(gcode_layer, gcode);
    }
}

LayerPlan& FffGcodeWriter::processLayer(const SliceDataStorage& storage, int layer_nr, unsigned int total_layers) const
{
    logDebug("GcodeWriter processing layer %i of %i\n", layer_nr, total_layers);

    int layer_thickness = getSettingInMicrons("layer_height");
    int64_t z;
    bool include_helper_parts = true;
    if (layer_nr < 0)
    {
#ifdef DEBUG
        assert(getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT && "negative layer_number means post-raft, pre-model layer!");
#endif // DEBUG
        const int filler_layer_count = Raft::getFillerLayerCount(storage);
        layer_thickness = Raft::getFillerLayerHeight(storage);
        z = Raft::getTotalThickness(storage) + (filler_layer_count + layer_nr + 1) * layer_thickness;
    }
    else
    {
        z = storage.meshes[0].layers[layer_nr].printZ; // stub default
        // find printZ of first actual printed mesh
        for (const SliceMeshStorage& mesh : storage.meshes)
        {
            if (layer_nr >= static_cast<int>(mesh.layers.size())
                || mesh.getSettingBoolean("support_mesh")
                || mesh.getSettingBoolean("anti_overhang_mesh")
                || mesh.getSettingBoolean("cutting_mesh")
                || mesh.getSettingBoolean("infill_mesh"))
            {
                continue;
            }
            z = mesh.layers[layer_nr].printZ; 
            layer_thickness = mesh.layers[layer_nr].thickness;
            break;
        }

        if (layer_nr == 0)
        {
            if (getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT)
            {
                include_helper_parts = false;
            }
        }
    }

    if (CommandSocket::isInstantiated())
    {
#pragma omp critical
        CommandSocket::getInstance()->sendOptimizedLayerInfo(layer_nr, z, layer_thickness);
    }

    bool avoid_other_parts = false;
    coord_t avoid_distance = 0; // minimal avoid distance is zero
    const std::vector<bool> extruder_is_used = storage.getExtrudersUsed();
    for (int extr_nr = 0; extr_nr < storage.meshgroup->getExtruderCount(); extr_nr++)
    {
        if (extruder_is_used[extr_nr])
        {
            ExtruderTrain* extr = storage.meshgroup->getExtruderTrain(extr_nr);

            if (extr->getSettingBoolean("travel_avoid_other_parts"))
            {
                avoid_other_parts = true;
                avoid_distance = std::max(avoid_distance, extr->getSettingInMicrons("travel_avoid_distance"));
            }
        }
    }

    coord_t max_inner_wall_width = 0;
    for (const SettingsBaseVirtual& mesh_settings : storage.meshes)
    {
        max_inner_wall_width = std::max(max_inner_wall_width, mesh_settings.getSettingInMicrons((mesh_settings.getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0")); 
        if (layer_nr == 0)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(mesh_settings.getSettingAsExtruderNr((mesh_settings.getSettingAsCount("wall_line_count") > 1) ? "wall_0_extruder_nr" : "wall_x_extruder_nr"));
            max_inner_wall_width *= train->getSettingAsRatio("initial_layer_line_width_factor");
        }
    }
    int64_t comb_offset_from_outlines = max_inner_wall_width * 2;

    assert(static_cast<int>(extruder_order_per_layer_negative_layers.size()) + layer_nr >= 0 && "Layer numbers shouldn't get more negative than there are raft/filler layers");
    
	const std::vector<unsigned int>& extruder_order =
        (layer_nr < 0)?
        extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + layer_nr]
        :
        extruder_order_per_layer[layer_nr];


    LayerPlan& gcode_layer = *new LayerPlan(*this, storage, layer_nr, z, layer_thickness, extruder_order.front(), fan_speed_layer_time_settings_per_extruder, getSettingAsCombingMode("retraction_combing"), comb_offset_from_outlines, avoid_other_parts, avoid_distance);

    if (include_helper_parts && layer_nr == 0)
    { // process the skirt or the brim of the starting extruder.
        int extruder_nr = gcode_layer.getExtruder();
        if (storage.skirt_brim[extruder_nr].size() > 0)
        {
            processSkirtBrim(storage, gcode_layer, extruder_nr);
        }
    }
    if (include_helper_parts)
    { // handle shield(s) first in a layer so that chances are higher that the other nozzle is wiped (for the ooze shield)
        processOozeShield(storage, gcode_layer);
        processDraftShield(storage, gcode_layer);
    }

    const unsigned int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const unsigned int support_bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    const unsigned int support_roof_skin_extruder_nr = getSettingAsIndex("support_roof_skin_extruder_nr");
    const unsigned int support_bottom_skin_extruder_nr = getSettingAsIndex("support_bottom_skin_extruder_nr");
    const unsigned int support_infill_extruder_nr = (layer_nr <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    for (unsigned int extruder_nr : extruder_order)
    {
        if (include_helper_parts
            && (extruder_nr == support_infill_extruder_nr || extruder_nr == support_roof_extruder_nr || extruder_nr == support_bottom_extruder_nr || extruder_nr == support_roof_skin_extruder_nr || extruder_nr == support_bottom_skin_extruder_nr))
        {
            addSupportToGCode(storage, gcode_layer, extruder_nr);
        }
        if (layer_nr >= 0)
        {
            const std::vector<unsigned int>& mesh_order = mesh_order_per_extruder[extruder_nr];
            for (unsigned int mesh_idx : mesh_order)
            {
                const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
                const PathConfigStorage::MeshPathConfigs& mesh_config = gcode_layer.configs_storage.mesh_configs[mesh_idx];
                if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
                {
                    assert(static_cast<int>(extruder_nr) == mesh.getSettingAsExtruderNr("wall_0_extruder_nr") && "mesh surface mode should always only be printed with the outer wall extruder!");
                    addMeshLayerToGCode_meshSurfaceMode(storage, mesh, mesh_config, gcode_layer);
                }
                else
                {
                    addMeshLayerToGCode(storage, mesh, extruder_nr, mesh_config, gcode_layer);
                }
            }
        }
        // ensure we print the prime tower with this extruder, because the next layer begins with this extruder!
        // If this is not performed, the next layer might get two extruder switches...
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
    }

    if (include_helper_parts)
    { // add prime tower if it hasn't already been added
        // print the prime tower if it hasn't been printed yet

        int prev_extruder = gcode_layer.getExtruder(); // most likely the same extruder as we are extruding with now
        addPrimeTower(storage, gcode_layer, prev_extruder);
    }
	
//	gcode_layer.optimizePaths(gcode.getPositionXY());

    return gcode_layer;
}

bool FffGcodeWriter::getExtruderNeedPrimeBlobDuringFirstLayer(const SliceDataStorage& storage, uint32_t extruder_nr) const
{
    bool need_prime_blob = false;
    switch (gcode.getFlavor())
    {
        case EGCodeFlavor::GRIFFIN:
            need_prime_blob = true;
            break;
        default:
            need_prime_blob = false; // TODO: change this once priming for other firmware types is implemented
            break;
    }

    // check the settings if the prime blob is disabled
    if (need_prime_blob)
    {
        const bool is_extruder_used_overall = storage.getExtrudersUsed()[extruder_nr];
        const bool extruder_prime_blob_enabled = storage.getExtruderPrimeBlobEnabled(extruder_nr);

        need_prime_blob = is_extruder_used_overall && extruder_prime_blob_enabled;
    }

    return need_prime_blob;
}

void FffGcodeWriter::processSkirtBrim(const SliceDataStorage& storage, LayerPlan& gcode_layer, unsigned int extruder_nr) const
{
    if (gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
    {
        return;
    }
    const Polygons& skirt_brim = storage.skirt_brim[extruder_nr];
    gcode_layer.setSkirtBrimIsPlanned(extruder_nr);
    if (skirt_brim.size() == 0)
    {
        return;
    }
    // Start brim close to the prime location
    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
    Point start_close_to;
    if (train->getSettingBoolean("prime_blob_enable"))
    {
        bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
        Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
        start_close_to = prime_pos_is_abs? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos;
    }
    else
    {
        start_close_to = gcode_layer.getLastPlannedPositionOrStartingPosition();
    }
    gcode_layer.addTravel(skirt_brim.back().closestPointTo(start_close_to));
    gcode_layer.addPolygonsByOptimizer(skirt_brim, gcode_layer.configs_storage.skirt_brim_config_per_extruder[extruder_nr]);
}

void FffGcodeWriter::processOozeShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    unsigned int layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }
    if (storage.oozeShield.size() > 0 && layer_nr < storage.oozeShield.size())
    {
        gcode_layer.addPolygonsByOptimizer(storage.oozeShield[layer_nr], gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
    }
}

void FffGcodeWriter::processDraftShield(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    unsigned int layer_nr = std::max(0, gcode_layer.getLayerNr());
    if (storage.draft_protection_shield.size() == 0)
    {
        return;
    }
    if (!getSettingBoolean("draft_shield_enabled"))
    {
        return;
    }
    if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::BRIM)
    {
        return; // ooze shield already generated by brim
    }

    if (getSettingAsDraftShieldHeightLimitation("draft_shield_height_limitation") == DraftShieldHeightLimitation::LIMITED)
    {
        const int draft_shield_height = getSettingInMicrons("draft_shield_height");
        const int layer_height_0 = getSettingInMicrons("layer_height_0");
        const int layer_height = getSettingInMicrons("layer_height");
        const unsigned int max_screen_layer = (unsigned int)((draft_shield_height - layer_height_0) / layer_height + 1);
        if (layer_nr > max_screen_layer)
        {
            return;
        }
    }

    gcode_layer.addPolygonsByOptimizer(storage.draft_protection_shield, gcode_layer.configs_storage.skirt_brim_config_per_extruder[0]); //TODO: Skirt and brim configuration index should correspond to draft shield extruder number.
}

void FffGcodeWriter::calculateExtruderOrderPerLayer(const SliceDataStorage& storage)
{
    unsigned int last_extruder;
    // set the initial extruder of this meshgroup
    if (FffProcessor::getInstance()->getMeshgroupNr() == 0)
    { // first meshgroup
        last_extruder = getStartExtruder(storage);
    }
    else
    {
        last_extruder = gcode.getExtruderNr();
    }

	int layer_thickness = getSettingInMicrons("layer_height");

    int nlayer_num = Raft::getTotalExtraLayers(storage);

    for (int layer_nr = -Raft::getTotalExtraLayers(storage); layer_nr < static_cast<int>(storage.print_layer_count); layer_nr++)
    {
        std::vector<std::vector<unsigned int>>& extruder_order_per_layer_here = (layer_nr < 0)? extruder_order_per_layer_negative_layers : extruder_order_per_layer;
        extruder_order_per_layer_here.push_back(getUsedExtrudersOnLayerExcludingStartingExtruder(storage, last_extruder, layer_nr));
        last_extruder = extruder_order_per_layer_here.back().back();
        extruder_prime_layer_nr[last_extruder] = std::min(extruder_prime_layer_nr[last_extruder], layer_nr);

		if (layer_nr >= 0 && storage.support.supportLayers[layer_nr].support_roof_interface.size() > 0 && storage.support.roof_interface_layer_thickness < layer_thickness)
		{
			unsigned int support_roof_extruder_nr = getSettingAsIndex("support_roof_skin_extruder_nr");
			if (support_roof_extruder_nr != extruder_order_per_layer[layer_nr][0])
			{
				std::vector<unsigned int>::iterator it = find(extruder_order_per_layer[layer_nr].begin(), extruder_order_per_layer[layer_nr].end(), support_roof_extruder_nr);
				if (it != extruder_order_per_layer[layer_nr].end())
				{
					int index = std::distance(extruder_order_per_layer[layer_nr].begin(), it);
					extruder_order_per_layer[layer_nr][index] = extruder_order_per_layer[layer_nr][0];
					extruder_order_per_layer[layer_nr][0] = support_roof_extruder_nr;

					last_extruder = extruder_order_per_layer.back().back();
					extruder_prime_layer_nr[last_extruder] = std::min(extruder_prime_layer_nr[last_extruder], layer_nr);
				}
			}
		}
    }
}

std::vector<unsigned int> FffGcodeWriter::getUsedExtrudersOnLayerExcludingStartingExtruder(const SliceDataStorage& storage, const unsigned int start_extruder, const int layer_nr) const
{
    unsigned int extruder_count = storage.getSettingAsCount("machine_extruder_count");
    assert(static_cast<int>(extruder_count) > 0);
    std::vector<unsigned int> ret;
    ret.push_back(start_extruder);
    std::vector<bool> extruder_is_used_on_this_layer = storage.getExtrudersUsed(layer_nr);

    // The outermost prime tower extruder is always used if there is a prime tower, apart on layers with negative index (e.g. for the raft)
    if (storage.getSettingBoolean("prime_tower_enable") && layer_nr >= 0 && 
        ((storage.getSettingAsPrimeTowerHeightType("prime_tower_height_type") == PrimeTowerHeightType::NORMAL && layer_nr <= storage.max_print_height_second_to_last_extruder)
            || (storage.getSettingAsPrimeTowerHeightType("prime_tower_height_type") == PrimeTowerHeightType::FULL)))
    {
        if (storage.getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED)
            extruder_is_used_on_this_layer[storage.primeTower.extruder_order[0]] = true;
        else
        {
            int outer_wall_index = storage.meshes[0].getSettingAsExtruderNr("wall_0_extruder_nr");
            if (outer_wall_index < 0 || outer_wall_index > extruder_count - 1)
                outer_wall_index = 0;

            extruder_is_used_on_this_layer[outer_wall_index] = true;
        }
    }

    // check if we are on the first layer
    if ((getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT && layer_nr == -Raft::getTotalExtraLayers(storage))
        || (getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT && layer_nr == 0))
    {
        // check if we need prime blob on the first layer
        for (unsigned int used_idx = 0; used_idx < extruder_is_used_on_this_layer.size(); used_idx++)
        {
            if (this->getExtruderNeedPrimeBlobDuringFirstLayer(storage, used_idx))
            {
                extruder_is_used_on_this_layer[used_idx] = true;
            }
        }
    }

    for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        if (extruder_nr == start_extruder)
        { // skip the current extruder, it's the one we started out planning
            continue;
        }
        if (!extruder_is_used_on_this_layer[extruder_nr])
        {
            continue;
        }
        ret.push_back(extruder_nr);
    }
    assert(ret.size() <= (size_t)extruder_count && "Not more extruders may be planned in a layer than there are extruders!");
    return ret;
}

std::vector<unsigned int> FffGcodeWriter::calculateMeshOrder(const SliceDataStorage& storage, int extruder_nr) const
{
    OrderOptimizer<unsigned int> mesh_idx_order_optimizer;

    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getExtruderIsUsed(extruder_nr))
        {
            const Mesh& mesh_data = storage.meshgroup->meshes[mesh_idx];
            const Point3 middle = (mesh_data.getAABB().min + mesh_data.getAABB().max) / 2;
            mesh_idx_order_optimizer.addItem(Point(middle.x, middle.y), mesh_idx);
        }
    }
    std::list<unsigned int> mesh_indices_order = mesh_idx_order_optimizer.optimize();
    std::list<unsigned int>::iterator starting_mesh_idx_it = mesh_indices_order.end();
    { // calculate starting_mesh_it
        const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
        const Point layer_start_position = Point(train->getSettingInMicrons("layer_start_x"), train->getSettingInMicrons("layer_start_y"));
        coord_t best_dist2 = std::numeric_limits<coord_t>::max();
        for (std::list<unsigned int>::iterator mesh_it = mesh_indices_order.begin(); mesh_it != mesh_indices_order.end(); ++mesh_it)
        {
            const unsigned int mesh_idx = *mesh_it;
            const Mesh& mesh = storage.meshgroup->meshes[mesh_idx];
            const Point3 middle3 = mesh.getAABB().getMiddle();
            const Point middle(middle3.x, middle3.y);
            const coord_t dist2 = vSize2(middle - layer_start_position);
            if (dist2 < best_dist2)
            {
                best_dist2 = dist2;
                starting_mesh_idx_it = mesh_it;
            }
        }
    }
    std::vector<unsigned int> ret;
    ret.reserve(mesh_indices_order.size());
    for (unsigned int mesh_order_nr = 0; mesh_order_nr < mesh_indices_order.size(); mesh_order_nr++)
    {
        if (starting_mesh_idx_it == mesh_indices_order.end())
        {
            starting_mesh_idx_it = mesh_indices_order.begin();
        }
        unsigned int mesh_order_idx = *starting_mesh_idx_it;
        const unsigned int mesh_idx = mesh_idx_order_optimizer.items[mesh_order_idx].second;
        ret.push_back(mesh_idx);
        ++starting_mesh_idx_it;
    }
    return ret;
}

void FffGcodeWriter::addMeshLayerToGCode_meshSurfaceMode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer)
    {
        return;
    }

    if (mesh.getSettingBoolean("anti_overhang_mesh") || mesh.getSettingBoolean("support_mesh"))
    {
        return;
    }

    setExtruder_addPrime(storage, gcode_layer, mesh.getSettingAsExtruderNr("wall_0_extruder_nr"));

    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];

    Polygons polygons;
    for(unsigned int partNr=0; partNr<layer->parts.size(); partNr++)
    {
        polygons.add(layer->parts[partNr].outline);
    }

    coord_t max_resolution = mesh.getSettingInMicrons("meshfix_maximum_resolution");
    coord_t max_deviation = mesh.getSettingInMicrons("meshfix_maximum_deviation");
    polygons.simplify(max_resolution, max_deviation);

    ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"), mesh.getSettingInMicrons("wall_line_width_0") * 2);
    const bool spiralize = mesh.getSettingBoolean("magic_spiralize");

    gcode_layer.addPolygonsByOptimizer(polygons, mesh_config.inset0_config, z_seam_config, mesh.getSettingInMicrons("wall_0_wipe_dist"), spiralize);

    addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
}

void FffGcodeWriter::addMeshOpenPolyLinesToGCode(const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    const SliceLayer* layer = &mesh.layers[gcode_layer.getLayerNr()];
    
    Polygons lines;
    for(ConstPolygonRef polyline : layer->openPolyLines)
    {
        for(unsigned int point_idx = 1; point_idx<polyline.size(); point_idx++)
        {
            Polygon p;
            p.add(polyline[point_idx-1]);
            p.add(polyline[point_idx]);
            lines.add(p);
        }
    }
    gcode_layer.addLinesByOptimizer(lines, mesh_config.inset0_config, SpaceFillType::PolyLines);
}

void FffGcodeWriter::addMeshLayerToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, LayerPlan& gcode_layer) const
{
    if (gcode_layer.getLayerNr() > mesh.layer_nr_max_filled_layer){
        return;
    }

    if (mesh.getSettingBoolean("anti_overhang_mesh") || mesh.getSettingBoolean("support_mesh")){
        return;
    }

    const SliceLayer& layer = mesh.layers[gcode_layer.getLayerNr()];

    if (layer.parts.size() == 0){
        return;
    }

    ZSeamConfig z_seam_config;
    if (mesh.isPrinted()) //"normal" meshes with walls, skin, infill, etc. get the traditional part ordering based on the z-seam settings.
    {
        z_seam_config = ZSeamConfig(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"), mesh.getSettingInMicrons("wall_line_width_0") * 2);
    }
    PathOrderOptimizer<const SliceLayerPart*> part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config);

    for (const SliceLayerPart& part : layer.parts)
    {
        part_order_optimizer.addPolygon(&part);
    }
    part_order_optimizer.optimize();

    for (const PathOrderPath<const SliceLayerPart*>& path : part_order_optimizer.paths)
    {
        addMeshPartToGCode(storage, mesh, extruder_nr, mesh_config, *path.vertices, gcode_layer);
    }

    const std::string extruder_identifier = (mesh.getSettingAsCount("roofing_layer_count") > 0) ? "roofing_extruder_nr" : "top_bottom_extruder_nr";
    if (extruder_nr == mesh.getSettingAsExtruderNr(extruder_identifier))
    {
        processIroning(mesh, layer, mesh_config.ironing_config, gcode_layer);
    }
    if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
    {
        addMeshOpenPolyLinesToGCode(mesh, mesh_config, gcode_layer);
    }
}

void FffGcodeWriter::addMeshPartToGCode(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, LayerPlan& gcode_layer) const
{
    int infill_angle = 45; // original default, this will get updated to an element from mesh->infill_angles
    if (mesh.infill_angles.size() > 0)
    {
        unsigned int combined_infill_layers = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(getSettingInMicrons("layer_height"), (coord_t)1)));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    
    int infill_line_distance = mesh.getSettingInMicrons("infill_line_distance");

	int infill_overlap = mesh.getSettingInMicrons("infill_overlap_mm");

    bool added_something = false;

    if (mesh.getSettingBoolean("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle);
    }

    added_something = added_something | processInsets(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, mesh.getZSeamHint());

    if (!mesh.getSettingBoolean("infill_before_walls"))
    {
        added_something = added_something | processInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle);
    }

    added_something = added_something | processSkin(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);

    //After a layer part, make sure the nozzle is inside the comb boundary, so we do not retract on the perimeter.
    if (added_something && (!getSettingBoolean("magic_spiralize") || static_cast<int>(gcode_layer.getLayerNr()) < mesh.getSettingAsCount("bottom_layers")))
    {
        int innermost_wall_line_width = mesh.getSettingInMicrons((mesh.getSettingAsCount("wall_line_count") > 1) ? "wall_line_width_x" : "wall_line_width_0");
        if (gcode_layer.getLayerNr() == 0)
        {
            innermost_wall_line_width *= mesh.getSettingAsRatio("initial_layer_line_width_factor");
        }
        gcode_layer.moveInsideCombBoundary(innermost_wall_line_width);
    }

    gcode_layer.setIsInside(false);
}

bool FffGcodeWriter::processInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, int infill_line_distance, int infill_overlap, int infill_angle) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.getSettingInMicrons("infill_offset_x"), mesh_middle.y + mesh.getSettingInMicrons("infill_offset_y"));
    if (mesh.getSettingBoolean("spaghetti_infill_enabled"))
    {
        return SpaghettiInfillPathGenerator::processSpaghettiInfill(storage, *this, gcode_layer, mesh, extruder_nr, mesh_config, part, infill_line_distance, infill_overlap, infill_angle, infill_origin);
    }
    else
    {
        bool added_something = processMultiLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
        added_something = added_something | processSingleLayerInfill(storage, gcode_layer, mesh, extruder_nr, mesh_config, part);
        return added_something;
    }
}

bool FffGcodeWriter::processMultiLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    const coord_t infill_line_distance = mesh.getSettingInMicrons("infill_line_distance");
    if (infill_line_distance <= 0)
    {
        return false;
    }
    coord_t max_resolution = mesh.getSettingInMicrons("meshfix_maximum_resolution");
    coord_t max_deviation = mesh.getSettingInMicrons("meshfix_maximum_deviation");
    AngleDegrees infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.

    if (!mesh.infill_angles.empty())
    {
        const size_t combined_infill_layers = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(mesh.getSettingInMicrons("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((gcode_layer.getLayerNr() / combined_infill_layers) % mesh.infill_angles.size());
    }
    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.getSettingInMicrons("infill_offset_x"), mesh_middle.y + mesh.getSettingInMicrons("infill_offset_y"));

    //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    bool added_something = false;
    for (unsigned int combine_idx = 1; combine_idx < part.infill_area_per_combine_per_density[0].size(); combine_idx++)
    {
        const coord_t infill_line_width = mesh_config.infill_config[combine_idx].getLineWidth();
        const EFillMethod infill_pattern = mesh.getSettingAsFillMethod("infill_pattern");
        const bool zig_zaggify_infill = mesh.getSettingBoolean("zig_zaggify_infill") || infill_pattern == EFillMethod::ZIG_ZAG;
        const bool connect_polygons = mesh.getSettingBoolean("connect_infill_polygons");
        const size_t infill_multiplier = mesh.getSettingAsCount("infill_multiplier");
        Polygons infill_polygons;
        Polygons infill_lines;
        std::vector<VariableWidthLines> infill_paths = part.infill_wall_toolpaths;
        for (size_t density_idx = part.infill_area_per_combine_per_density.size() - 1; (int)density_idx >= 0; density_idx--)
        { // combine different density infill areas (for gradual infill)
            size_t density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
            coord_t infill_line_distance_here = infill_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
            coord_t infill_shift = infill_line_distance_here / 2;
            if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || infill_pattern == EFillMethod::CROSS || infill_pattern == EFillMethod::CROSS_3D)
            {
                infill_line_distance_here /= 2;
            }
            constexpr size_t wall_line_count = 0; // wall toolpaths are when gradual infill areas are determined
            constexpr coord_t infill_overlap = 0; // Overlap is handled when the wall toolpaths are generated
            constexpr bool skip_stitching = false;
            constexpr bool connected_zigzags = false;
            constexpr bool use_endpieces = true;
            constexpr bool skip_some_zags = false;
            constexpr size_t zag_skip_count = 0;

            Infill infill_comp(infill_pattern, zig_zaggify_infill, connect_polygons,
                part.infill_area_per_combine_per_density[density_idx][combine_idx], infill_line_width,
                infill_line_distance_here, infill_overlap, infill_multiplier, infill_angle,
                gcode_layer.z, infill_shift, max_resolution, max_deviation, wall_line_count,
                infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count,
                mesh.getSettingInMicrons("cross_infill_pocket_size"));
            infill_comp.generate(infill_paths, infill_polygons, infill_lines, mesh, mesh.cross_fill_provider, &mesh);
        }
        if (!infill_lines.empty() || !infill_polygons.empty())
        {
            added_something = true;
            setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object

            if (!infill_polygons.empty())
            {
                constexpr bool force_comb_retract = false;
                gcode_layer.addTravel(infill_polygons[0][0], force_comb_retract);
                gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[combine_idx]);
            }

            if (!infill_lines.empty())
            {
                std::optional<Point> near_start_location;
                if (mesh.getSettingBoolean("infill_randomize_start_location"))
                {
                    srand(gcode_layer.getLayerNr());
                    near_start_location = infill_lines[rand() % infill_lines.size()][0];
                }
                const bool enable_travel_optimization = mesh.getSettingBoolean("infill_enable_travel_optimization");
                gcode_layer.addLinesByOptimizer(infill_lines,
                                                mesh_config.infill_config[combine_idx],
                                                zig_zaggify_infill ? SpaceFillType::PolyLines : SpaceFillType::Lines,
                                                enable_travel_optimization,
                                                /*wipe_dist = */ 0,
                                                /* flow = */ 1.0,
                                                near_start_location);
            }
        }
    }
    return added_something;
}

bool FffGcodeWriter::processSingleLayerInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    if (extruder_nr != mesh.getSettingAsExtruderNr("infill_extruder_nr"))
    {
        return false;
    }
    const auto infill_line_distance = mesh.getSettingInMicrons("infill_line_distance");
    if (infill_line_distance == 0 || part.infill_area_per_combine_per_density[0].size() == 0)
    {
        return false;
    }
    bool added_something = false;
    const unsigned int infill_line_width = mesh_config.infill_config[0].getLineWidth();
        
    //Combine the 1 layer thick infill with the top/bottom skin and print that as one thing.
    Polygons infill_polygons;
    std::vector<std::vector<VariableWidthLines>> wall_tool_paths; //all wall toolpaths binned by inset_idx (inner) and by density_idx (outer).
    Polygons infill_lines;

    EFillMethod pattern = mesh.getSettingAsFillMethod("infill_pattern");
    const bool zig_zaggify_infill = mesh.getSettingBoolean("zig_zaggify_infill") || pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = mesh.getSettingBoolean("connect_infill_polygons");
    const auto infill_overlap = mesh.getSettingInMicrons("infill_overlap_mm");
    const auto infill_multiplier = mesh.getSettingAsCount("infill_multiplier");
    const auto wall_line_count = mesh.getSettingAsCount("infill_wall_line_count");
    const size_t last_idx = part.infill_area_per_combine_per_density.size() - 1;
    const auto max_resolution = mesh.getSettingInMicrons("meshfix_maximum_resolution");
    const auto max_deviation = mesh.getSettingInMicrons("meshfix_maximum_deviation");
    AngleDegrees infill_angle = 45; //Original default. This will get updated to an element from mesh->infill_angles.

    if (!mesh.infill_angles.empty())
    {
        const size_t combined_infill_layers = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(mesh.getSettingInMicrons("layer_height"), coord_t(1))));
        infill_angle = mesh.infill_angles.at((static_cast<size_t>(gcode_layer.getLayerNr()) / combined_infill_layers) % mesh.infill_angles.size());
    }

    const Point3 mesh_middle = mesh.bounding_box.getMiddle();
    const Point infill_origin(mesh_middle.x + mesh.getSettingInMicrons("infill_offset_x"), mesh_middle.y + mesh.getSettingInMicrons("infill_offset_y"));
    
    auto get_cut_offset = [](const bool zig_zaggify, const coord_t line_width, const size_t line_count)
    {
        if (zig_zaggify)
        {
            return -line_width / 2 - static_cast<coord_t>(line_count) * line_width - 5;
        }
        return -static_cast<coord_t>(line_count) * line_width;
    };

    Polygons sparse_in_outline = part.infill_area_per_combine_per_density[last_idx][0];

    // if infill walls are required below the boundaries of skin regions above, partition the infill along the
    // boundary edge
    Polygons infill_below_skin;
    Polygons infill_not_below_skin;
    const bool hasSkinEdgeSupport = partitionInfillBySkinAbove(infill_below_skin, infill_not_below_skin, gcode_layer, mesh, part, infill_line_width);

    const auto pocket_size = mesh.getSettingInMicrons("cross_infill_pocket_size");
    constexpr bool skip_stitching = false;
    constexpr bool connected_zigzags = false;
    const bool use_endpieces = part.infill_area_per_combine_per_density.size() == 1; //Only use endpieces when not using gradual infill, since they will then overlap.
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;

    for (size_t density_idx = last_idx; static_cast<int>(density_idx) >= 0; density_idx--)
    {
        // Only process dense areas when they're initialized
        if (part.infill_area_per_combine_per_density[density_idx][0].empty())
        {
            continue;
        }

        Polygons infill_lines_here;
        Polygons infill_polygons_here;

        // the highest density infill combines with the next to create a grid with density_factor 1
        int infill_line_distance_here = infill_line_distance << (density_idx + 1);
        int infill_shift = infill_line_distance_here / 2;

        /* infill shift explanation: [>]=shift ["]=line_dist

         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>"""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>"""""""""
         :       |       :       |       :       |       :       |         > furthest from top
         :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |     > further from top
         : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | |   > near top
         >>>>>>>>"""""""""""""""""
         */
         //All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2.
        if (density_idx == part.infill_area_per_combine_per_density.size() - 1 || pattern == EFillMethod::CROSS || pattern == EFillMethod::CROSS_3D)
        {
            /* the least dense infill should fill up all remaining gaps
             :       |       :       |       :       |       :       |       :  > furthest from top
             :   |   |   |   :   |   |   |   :   |   |   |   :   |   |   |   :  > further from top
             : | | | | | | | : | | | | | | | : | | | | | | | : | | | | | | | :  > near top
               .   .     .       .           .               .       .       .
               :   :     :       :           :               :       :       :
               `"""'     `"""""""'           `"""""""""""""""'       `"""""""'
                                                                         ^   new line distance for lowest density infill
                                                   ^ infill_line_distance_here for lowest density infill up till here
                             ^ middle density line dist
                 ^   highest density line dist*/

            //All of that doesn't hold for the Cross patterns; they should just always be multiplied by 2 for every density index.
            infill_line_distance_here /= 2;
        }
        Polygons in_outline = part.infill_area_per_combine_per_density[density_idx][0];

        if (hasSkinEdgeSupport)
        {
            // infill region with skin above has to have at least one infill wall line
            const size_t min_skin_below_wall_count = wall_line_count > 0 ? wall_line_count : 1;
            const size_t skin_below_wall_count = density_idx == last_idx ? min_skin_below_wall_count : 0;
            wall_tool_paths.emplace_back(std::vector<VariableWidthLines>());
            const coord_t overlap = infill_overlap - (density_idx == last_idx ? 0 : wall_line_count * infill_line_width);
            Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, infill_below_skin, infill_line_width,
                infill_line_distance_here, overlap, infill_multiplier, infill_angle, gcode_layer.z,
                infill_shift, max_resolution, max_deviation, skin_below_wall_count, infill_origin,
                skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
            infill_comp.generate(wall_tool_paths.back(), infill_polygons, infill_lines, mesh, mesh.cross_fill_provider, &mesh);

            // Fixme: CURA-7848 for libArachne.
            if (density_idx < last_idx)
            {
                const coord_t cut_offset =
                    get_cut_offset(zig_zaggify_infill, infill_line_width, min_skin_below_wall_count);
                Polygons tool = infill_below_skin.offset(static_cast<int>(cut_offset));
                infill_lines_here = tool.intersectionPolyLines(infill_lines_here);
            }
            infill_lines.add(infill_lines_here);
            // normal processing for the infill that isn't below skin
            in_outline = infill_not_below_skin;
            if (density_idx == last_idx)
            {
                sparse_in_outline = infill_not_below_skin;
            }
        }
        const coord_t circumference = in_outline.polygonLength();
        //Originally an area of 0.4*0.4*2 (2 line width squares) was found to be a good threshold for removal.
        //However we found that this doesn't scale well with polygons with larger circumference (https://github.com/Ultimaker/Cura/issues/3992).
        //Given that the original test worked for approximately 2x2cm models, this scaling by circumference should make it work for any size.
        constexpr double minimum_small_area_factor = 0.4 * 0.4 / 40000;
        const double minimum_small_area = minimum_small_area_factor * circumference;

        // This is only for density infill, because after generating the infill might appear unnecessary infill on walls
        // especially on vertical surfaces
        in_outline.removeSmallAreas(minimum_small_area);

        const size_t wall_line_count_here = part.getTopAndBottomLayerInfillWallCount(); // Wall toolpaths were generated in generateGradualInfill for the sparsest density, denser parts don't have walls by default
        constexpr coord_t overlap = 0; // overlap is already applied for the sparsest density in the generateGradualInfill

        wall_tool_paths.emplace_back();
        Infill infill_comp(pattern, zig_zaggify_infill, connect_polygons, in_outline, infill_line_width,
            infill_line_distance_here, overlap, infill_multiplier, infill_angle, gcode_layer.z,
            infill_shift, max_resolution, max_deviation, wall_line_count_here, infill_origin,
            skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
        infill_comp.generate(wall_tool_paths.back(), infill_polygons, infill_lines, mesh, mesh.cross_fill_provider, &mesh);

        // Fixme: CURA-7848 for libArachne.
        if (density_idx < last_idx)
        {
            const coord_t cut_offset = get_cut_offset(zig_zaggify_infill, infill_line_width, wall_line_count);
            Polygons tool = sparse_in_outline.offset(static_cast<int>(cut_offset));
            infill_lines_here = tool.intersectionPolyLines(infill_lines_here);
        }
        infill_lines.add(infill_lines_here);
        infill_polygons.add(infill_polygons_here);
    }
    wall_tool_paths.emplace_back(part.infill_wall_toolpaths); //The extra infill walls were generated separately. Add these too.
    const bool walls_generated = std::any_of(wall_tool_paths.cbegin(), wall_tool_paths.cend(), [](const std::vector<VariableWidthLines>& tp) { return !tp.empty(); });
    if (!infill_lines.empty() || !infill_polygons.empty() || walls_generated)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); //going to print stuff inside print object
        std::optional<Point> near_start_location;
        if (mesh.getSettingBoolean("infill_randomize_start_location"))
        {
            srand(gcode_layer.getLayerNr());
            if (!infill_lines.empty())
            {
                near_start_location = infill_lines[rand() % infill_lines.size()][0];
            }
            else if (!infill_polygons.empty())
            {
                PolygonRef start_poly = infill_polygons[rand() % infill_polygons.size()];
                near_start_location = start_poly[rand() % start_poly.size()];
            }
            else //So walls_generated must be true.
            {
                std::vector<VariableWidthLines>* start_paths = &wall_tool_paths[rand() % wall_tool_paths.size()];
                while (start_paths->empty()) //We know for sure (because walls_generated) that one of them is not empty. So randomise until we hit it. Should almost always be very quick.
                {
                    start_paths = &wall_tool_paths[rand() % wall_tool_paths.size()];
                }
                near_start_location = (*start_paths)[0][0].junctions[0].p;
            }
        }
        if (walls_generated)
        {
            for (const std::vector<VariableWidthLines>& tool_paths : wall_tool_paths)
            {
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                const ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"), mesh_config.infill_config[0].getLineWidth() * 2);

                InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer,  mesh, extruder_nr,
                    mesh_config.infill_config[0], mesh_config.infill_config[0], mesh_config.infill_config[0], mesh_config.infill_config[0],
                    retract_before_outer_wall, wipe_dist, wipe_dist, extruder_nr, extruder_nr, z_seam_config, tool_paths);
                added_something |= wall_orderer.addToLayer();
            }
        }
        if (!infill_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            // start the infill polygons at the nearest vertex to the current location
            gcode_layer.addTravel(PolygonUtils::findNearestVert(gcode_layer.getLastPlannedPositionOrStartingPosition(), infill_polygons).p(), force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(infill_polygons, mesh_config.infill_config[0], ZSeamConfig(), 0, false, 1.0_r, false, false, near_start_location);
        }
        const bool enable_travel_optimization = mesh.getSettingBoolean("infill_enable_travel_optimization");
        if (pattern == EFillMethod::GRID
            || pattern == EFillMethod::LINES
            || pattern == EFillMethod::TRIANGLES
            || pattern == EFillMethod::CUBIC
            || pattern == EFillMethod::TETRAHEDRAL
            || pattern == EFillMethod::QUARTER_CUBIC
            || pattern == EFillMethod::CUBICSUBDIV)
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], SpaceFillType::Lines, enable_travel_optimization
                , mesh.getSettingInMicrons("infill_wipe_dist"), /*float_ratio = */ 1.0, near_start_location);
        }
        else
        {
            gcode_layer.addLinesByOptimizer(infill_lines, mesh_config.infill_config[0], (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines, enable_travel_optimization
                , /* wipe_dist = */ 0, /*float_ratio = */ 1.0, near_start_location);
        }
    }
    return true;
}
void FffGcodeWriter::processSpiralizedWall(const SliceDataStorage& storage, LayerPlan& gcode_layer, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, const SliceMeshStorage& mesh) const
{
    if (part.spiral_wall.empty())
    {
        // wall doesn't have usable outline
        return;
    }
    const ClipperLib::Path* last_wall_outline = &*part.spiral_wall[0]; // default to current wall outline
    int last_seam_vertex_idx = -1; // last layer seam vertex index
    int layer_nr = gcode_layer.getLayerNr();
    if (layer_nr > 0)
    {
        if (storage.spiralize_wall_outlines[layer_nr - 1] != nullptr)
        {
            // use the wall outline from the previous layer
            last_wall_outline = &*(*storage.spiralize_wall_outlines[layer_nr - 1])[0];
            // and the seam vertex index pre-computed for that layer
            last_seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr - 1];
        }
    }
    const bool is_bottom_layer = (layer_nr == mesh.getSettingAsCount("bottom_layers"));
    const bool is_top_layer = ((size_t)layer_nr == (storage.spiralize_wall_outlines.size() - 1) || storage.spiralize_wall_outlines[layer_nr + 1] == nullptr);
    const int seam_vertex_idx = storage.spiralize_seam_vertex_indices[layer_nr]; // use pre-computed seam vertex index for current layer

    // output a wall slice that is interpolated between the last and current walls
    for (const ConstPolygonRef& wall_outline : part.spiral_wall)
    {
        gcode_layer.spiralizeWallSlice(mesh_config.inset0_config, wall_outline, ConstPolygonRef(*last_wall_outline), seam_vertex_idx, last_seam_vertex_idx, is_top_layer, is_bottom_layer);
    }
}

bool FffGcodeWriter::processInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part, Point z_seam_pos) const
{
    bool added_something = false;
    if (extruder_nr != mesh.getSettingAsExtruderNr("wall_0_extruder_nr") && extruder_nr != mesh.getSettingAsExtruderNr("wall_x_extruder_nr"))
    {
        return added_something;
    }

    if (mesh.getSettingAsCount("wall_line_count") <= 0)
    {
        return added_something;
    }
    bool spiralize = false;
    if (getSettingBoolean("magic_spiralize"))
    {
        const size_t initial_bottom_layers = mesh.getSettingAsCount("initial_bottom_layers");
        const int layer_nr = gcode_layer.getLayerNr();
        if ((layer_nr < static_cast<LayerIndex>(initial_bottom_layers) && part.wall_toolpaths.empty())
            || (layer_nr >= static_cast<LayerIndex>(initial_bottom_layers) && part.spiral_wall.empty())){
            //nothing to do
            return false;
        }
        if (gcode_layer.getLayerNr() >= static_cast<LayerIndex>(initial_bottom_layers))
        {
            spiralize = true;
        }
        if (spiralize && gcode_layer.getLayerNr() == static_cast<LayerIndex>(initial_bottom_layers) && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr"))
        { // on the last normal layer first make the outer wall normally and then start a second outer wall from the same hight, but gradually moving upward
            added_something = true;
            setExtruder_addPrime(storage, gcode_layer, extruder_nr);
            gcode_layer.setIsInside(true); // going to print stuff inside print object
            // start this first wall at the same vertex the spiral starts
            const ConstPolygonRef spiral_inset = part.spiral_wall[0];
            const size_t spiral_start_vertex = storage.spiralize_seam_vertex_indices[initial_bottom_layers];
            if (spiral_start_vertex < spiral_inset.size())
            {
                gcode_layer.addTravel(spiral_inset[spiral_start_vertex]);
            }
            int wall_0_wipe_dist(0);
            gcode_layer.addPolygonsByOptimizer(part.spiral_wall, mesh_config.inset0_config, ZSeamConfig(), wall_0_wipe_dist);
        }
    }
    // for non-spiralized layers, determine the shape of the unsupported areas below this part
    if (!spiralize && gcode_layer.getLayerNr() > 0)
    {
        // accumulate the outlines of all of the parts that are on the layer below
        Polygons outlines_below;
        AABB boundaryBox(part.outline);
        for (const SliceMeshStorage& m : storage.meshes){
            if (m.isPrinted()){
                for (const SliceLayerPart& prevLayerPart : m.layers[gcode_layer.getLayerNr() - 1].parts){
                    if (boundaryBox.hit(prevLayerPart.boundaryBox)){
                        outlines_below.add(prevLayerPart.outline);
                    }
                }
            }
        }
        const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();

        // if support is enabled, add the support outlines also so we don't generate bridges over support
        if (storage.getSettingBoolean("support_enable"))
        {
            const coord_t z_distance_top = mesh.getSettingInMicrons("support_top_distance");
            const size_t z_distance_top_layers = std::max(uint64_t(0), round_up_divide(z_distance_top, layer_height)) + 1;
            const int support_layer_nr = gcode_layer.getLayerNr() - z_distance_top_layers;
            if (support_layer_nr > 0)
            {
                const SupportLayer& support_layer = storage.support.supportLayers[support_layer_nr];
                if (!support_layer.support_roof.empty())
                {
                    AABB support_roof_bb(support_layer.support_roof);
                    if (boundaryBox.hit(support_roof_bb))
                    {
                        outlines_below.add(support_layer.support_roof);
                    }
                }
                else
                {
                    for (const SupportInfillPart& support_part : support_layer.support_infill_parts)
                    {
                        AABB support_part_bb(support_part.getInfillArea());
                        if (boundaryBox.hit(support_part_bb))
                        {
                            outlines_below.add(support_part.getInfillArea());
                        }
                    }
                }
            }
        }
        const int half_outer_wall_width = mesh_config.inset0_config.getLineWidth() / 2;
        // remove those parts of the layer below that are narrower than a wall line width as they will not be printed
        outlines_below = outlines_below.offset(-half_outer_wall_width).offset(half_outer_wall_width);

        if (mesh.getSettingBoolean("bridge_settings_enabled"))
        {
            // max_air_gap is the max allowed width of the unsupported region below the wall line
            // if the unsupported region is wider than max_air_gap, the wall line will be printed using bridge settings
            const coord_t max_air_gap = half_outer_wall_width;
            // subtract the outlines of the parts below this part to give the shapes of the unsupported regions and then
            // shrink those shapes so that any that are narrower than two times max_air_gap will be removed
            Polygons compressed_air(part.outline.difference(outlines_below).offset(-max_air_gap));
            // now expand the air regions by the same amount as they were shrunk plus half the outer wall line width
            // which is required because when the walls are being generated, the vertices do not fall on the part's outline
            // but, instead, are 1/2 a line width inset from the outline
            gcode_layer.setBridgeWallMask(compressed_air.offset(max_air_gap + half_outer_wall_width));
        }
        else
        {
            // clear to disable use of bridging settings
            gcode_layer.setBridgeWallMask(Polygons());
        }
        const AngleDegrees overhang_angle = mesh.getSettingInAngleDegrees("wall_overhang_angle");
        if (overhang_angle >= 90)
        {
            // clear to disable overhang detection
            gcode_layer.setOverhangMask(Polygons());
        }
        else
        {
            // the overhang mask is set to the area of the current part's outline minus the region that is considered to be supported
            // the supported region is made up of those areas that really are supported by either model or support on the layer below
            // expanded to take into account the overhang angle, the greater the overhang angle, the larger the supported area is
            // considered to be
            const coord_t overhang_width = layer_height * std::tan(overhang_angle / (180 / M_PI));
            Polygons overhang_region = part.outline.offset(-half_outer_wall_width).difference(outlines_below.offset(10 + overhang_width - half_outer_wall_width)).offset(10);
            gcode_layer.setOverhangMask(overhang_region);
        }
    }
    else
    {
        // clear to disable use of bridging settings
        gcode_layer.setBridgeWallMask(Polygons());
        // clear to disable overhang detection
        gcode_layer.setOverhangMask(Polygons());
    }

    if (spiralize && extruder_nr == mesh.getSettingAsExtruderNr("wall_0_extruder_nr") && !part.spiral_wall.empty())
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object

        // Only spiralize the first part in the mesh, any other parts will be printed using the normal, non-spiralize codepath.
        // This sounds weird but actually does the right thing when you have a model that has multiple parts at the bottom that merge into
        // one part higher up. Once all the parts have merged, layers above that level will be spiralized
        if (&mesh.layers[gcode_layer.getLayerNr()].parts[0] == &part)
        {
            processSpiralizedWall(storage, gcode_layer, mesh_config, part, mesh);
        }
        else
        {
            //Print the spiral walls of other parts as single walls without Z gradient.
            gcode_layer.addWalls(part.spiral_wall, mesh, mesh_config.inset0_config, mesh_config.inset0_config);
        }
    }
    else
    {
        //Main case: Optimize the insets with the InsetOrderOptimizer.
        const coord_t wall_x_wipe_dist = 0;
        const ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"), mesh.getSettingInMicrons("wall_line_width_0") * 2);
        
        InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, mesh, extruder_nr,
            mesh_config.inset0_config, mesh_config.insetX_config, mesh_config.bridge_inset0_config, mesh_config.bridge_insetX_config,
            mesh.getSettingBoolean("travel_retract_before_outer_wall"), mesh.getSettingInMillimeters("wall_0_wipe_dist"), wall_x_wipe_dist,
            mesh.getSettingAsExtruderNr("wall_0_extruder_nr"), mesh.getSettingAsExtruderNr("wall_x_extruder_nr"), z_seam_config, part.wall_toolpaths);
        added_something |= wall_orderer.addToLayer();
    }
    return added_something;
}

std::optional<Point> FffGcodeWriter::getSeamAvoidingLocation(const Polygons& filling_part, int filling_angle, Point last_position) const
{
    if (filling_part.empty())
    {
        return std::optional<Point>();
    }
    // start with the BB of the outline
    AABB skin_part_bb(filling_part);
    PointMatrix rot((double)((-filling_angle + 90) % 360)); // create a matrix to rotate a vector so that it is normal to the skin angle
    const Point bb_middle = skin_part_bb.getMiddle();
    // create a vector from the middle of the BB whose length is such that it can be rotated
    // around the middle of the BB and the end will always be a long way outside of the part's outline
    // and rotate the vector so that it is normal to the skin angle
    const Point vec = rot.apply(Point(0, vSize(skin_part_bb.max - bb_middle) * 100));
    // find the vertex in the outline that is closest to the end of the rotated vector
    const PolygonsPointIndex pa = PolygonUtils::findNearestVert(bb_middle + vec, filling_part);
    // and find another outline vertex, this time using the vector + 180 deg
    const PolygonsPointIndex pb = PolygonUtils::findNearestVert(bb_middle - vec, filling_part);
    if (!pa.initialized() || !pb.initialized())
    {
        return std::optional<Point>();
    }
    // now go to whichever of those vertices that is closest to where we are now
    if (vSize2(pa.p() - last_position) < vSize2(pb.p() - last_position))
    {
        bool bs_arg = true;
        return std::optional<Point>(bs_arg, pa.p());
    }
    else
    {
        bool bs_arg = true;
        return std::optional<Point>(bs_arg, pb.p());
    }
}

bool FffGcodeWriter::processSkin(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const size_t extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SliceLayerPart& part) const
{
    const size_t top_bottom_extruder_nr = mesh.getSettingAsExtruderNr("top_bottom_extruder_nr");
    const size_t roofing_extruder_nr = mesh.getSettingAsExtruderNr("roofing_extruder_nr");
    const size_t flooring_extruder_nr = mesh.getSettingAsExtruderNr("flooring_extruder_nr");
    const size_t wall_0_extruder_nr = mesh.getSettingAsExtruderNr("wall_0_extruder_nr");
    const size_t roofing_layer_count  = std::min(mesh.getSettingAsCount("roofing_layer_count"), mesh.getSettingAsCount("top_layers"));
    const size_t flooring_layer_count = std::min(mesh.getSettingAsCount("flooring_layer_count"), mesh.getSettingAsCount("bottom_layers"));
    if (extruder_nr != top_bottom_extruder_nr && extruder_nr != wall_0_extruder_nr
        && (extruder_nr != roofing_extruder_nr || roofing_layer_count <= 0)
        && (extruder_nr != flooring_extruder_nr || flooring_layer_count<=0))
    {
        return false;
    }
    bool added_something = false;

    PathOrderOptimizer<const SkinPart*> part_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (const SkinPart& skin_part : part.skin_parts)
    {
        part_order_optimizer.addPolygon(&skin_part);
    }
    part_order_optimizer.optimize();

    for (const PathOrderPath<const SkinPart*>& path : part_order_optimizer.paths)
    {
        const SkinPart& skin_part = *path.vertices;

        added_something = added_something |
            processSkinPart(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part);
    }

    return added_something;
}

bool FffGcodeWriter::processSkinPart(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part) const
{
    bool added_something = false;

    // add roofing
    processRoofing(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);

    // add normal skinfill
    processTopBottom(storage, gcode_layer, mesh, extruder_nr, mesh_config, skin_part, added_something);
    
    return added_something;
}

void FffGcodeWriter::processSkinInsets(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
    const int skin_extruder_nr = mesh.getSettingAsExtruderNr("top_bottom_extruder_nr");
    // add skin walls aka skin perimeters
    if (extruder_nr == skin_extruder_nr)
    {
        for (const Polygons& skin_perimeter : skin_part.insets)
        {
            if (skin_perimeter.size() > 0)
            {
                added_something = true;
                setExtruder_addPrime(storage, gcode_layer, extruder_nr);
                gcode_layer.setIsInside(true); // going to print stuff inside print object

                if (skin_part.skin_position_type == SkinPositionType::TOP) {
                    gcode_layer.addPolygonsByOptimizer(skin_perimeter, mesh_config.top_skin_config);
                }
                else {
                    gcode_layer.addPolygonsByOptimizer(skin_perimeter, mesh_config.bottom_skin_config);
                }
            }
        }
    }
}

void FffGcodeWriter::processRoofing(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
    const int roofing_extruder_nr = mesh.getSettingAsExtruderNr("roofing_extruder_nr");
    if (extruder_nr != roofing_extruder_nr)
    {
        return;
    }

    EFillMethod pattern = mesh.getSettingAsFillMethod("roofing_pattern");

    int roofing_angle = 45;
    if (mesh.roofing_angles.size() > 0)
    {
        roofing_angle = mesh.roofing_angles.at(gcode_layer.getLayerNr() % mesh.roofing_angles.size());
    }

    const double skin_density = 1.0;
    const coord_t skin_overlap = 0; // skinfill already expanded over the roofing areas; don't overlap with perimeters
    const bool monotonic = mesh.getSettingBoolean("roofing_monotonic");

    processSkinPrintFeature(storage, gcode_layer, mesh, mesh_config, roofing_extruder_nr, skin_part.roofing_fill, mesh_config.roofing_infill_config, pattern, roofing_angle, skin_overlap, skin_density, monotonic, added_something);
}

void FffGcodeWriter::processTopBottom(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const int extruder_nr, const PathConfigStorage::MeshPathConfigs& mesh_config, const SkinPart& skin_part, bool& added_something) const
{
    if (skin_part.skin_fill.empty()){
        return;
    }

    const int top_bottom_extruder_nr = mesh.getSettingAsExtruderNr("top_bottom_extruder_nr");
    if (extruder_nr != top_bottom_extruder_nr){
        return;
    }

    const unsigned layer_nr = gcode_layer.getLayerNr();

    EFillMethod pattern = (layer_nr == 0)?
        mesh.getSettingAsFillMethod("top_bottom_pattern_0") :
        mesh.getSettingAsFillMethod("top_bottom_pattern");

    int skin_angle = 45;
    if (mesh.skin_angles.size() > 0)
    {
        skin_angle = mesh.skin_angles.at(layer_nr % mesh.skin_angles.size());
    }

    // generate skin_polygons and skin_lines
    const GCodePathConfig* skin_config = &mesh_config.top_skin_config;
    double skin_density = 1.0;
	coord_t skin_overlap = mesh.getSettingInMicrons("skin_top_overlap_mm");
    if (skin_part.skin_position_type == SkinPositionType::BOTTOM)
    {
        skin_overlap = mesh.getSettingInMicrons("skin_bottom_overlap_mm");
        skin_config = &mesh_config.bottom_skin_config;
    }

    const coord_t more_skin_overlap = std::max(skin_overlap, (coord_t)(mesh_config.insetX_config.getLineWidth() / 2));

    const bool bridge_settings_enabled = mesh.getSettingBoolean("bridge_settings_enabled");
    const bool bridge_enable_more_layers = bridge_settings_enabled && mesh.getSettingBoolean("bridge_enable_more_layers");
    const float support_threshold = bridge_settings_enabled ? mesh.getSettingBoolean("bridge_skin_support_threshold") : 0.0f;
    const size_t bottom_layers = mesh.getSettingAsCount("bottom_layers");

    // if support is enabled, consider the support outlines so we don't generate bridges over support

    int support_layer_nr = -1;
    const SupportLayer* support_layer = nullptr;

    if (mesh.getSettingBoolean("support_enable"))
    {
        const coord_t layer_height = mesh_config.inset0_config.getLayerThickness();
        const coord_t z_distance_top = mesh.getSettingInMicrons("support_top_distance");
        const size_t z_distance_top_layers = round_up_divide(z_distance_top, layer_height) + 1;
        support_layer_nr = layer_nr - z_distance_top_layers;
    }

    //helper function that detects skin regions that have no support and modifies their print settings (config, line angle, density, etc.)
    auto handle_bridge_skin = [&](const int bridge_layer, const GCodePathConfig* config, const float density) // bridge layer = 1, 2 or 3
    {
        if (support_layer_nr >= (bridge_layer - 1))
        {
            support_layer = &storage.support.supportLayers[support_layer_nr - (bridge_layer -1)];
        }
        Polygons supported_skin_part_regions;
        const int angle = bridgeAngle(mesh, skin_part.skin_fill, storage, layer_nr, bridge_layer, support_layer, supported_skin_part_regions);
        if (angle > -1 || (support_threshold > 0 && (supported_skin_part_regions.area() / (skin_part.skin_fill.area() + 1) < support_threshold)))
        {
            if (angle > -1)
            {
                switch (bridge_layer)
                {
                default:
                case 1:
                    skin_angle = angle;
                    break;
                case 2:
                    if (bottom_layers > 2){
                        // orientate second bridge skin at +45 deg to first
                        skin_angle = angle + 45;
                    }
                    else{
                        // orientate second bridge skin at 90 deg to first
                        skin_angle = angle + 90;
                    }
                    break;

                case 3:
                    // orientate third bridge skin at 135 (same result as -45) deg to first
                    skin_angle = angle + 135;
                    break;
                }
            }
            pattern = EFillMethod::LINES; // force lines pattern when bridging
            if (bridge_settings_enabled)
            {
                skin_config = config;
                skin_overlap = more_skin_overlap;
                skin_density = density;
            }
            return true;
        }

        return false;
    };

    bool is_bridge_skin = false;
    if (layer_nr > 0)
    {
        is_bridge_skin = handle_bridge_skin(1, &mesh_config.top_bridge_skin_config, mesh.getSettingAsRatio("bridge_skin_density"));
    }
    if (bridge_enable_more_layers && !is_bridge_skin && layer_nr > 1 && bottom_layers > 1)
    {
        is_bridge_skin = handle_bridge_skin(2, &mesh_config.top_bridge_skin_config2, mesh.getSettingAsRatio("bridge_skin_density_2"));

        if (!is_bridge_skin && layer_nr > 2 && bottom_layers > 2)
        {
            is_bridge_skin = handle_bridge_skin(3, &mesh_config.top_bridge_skin_config3, mesh.getSettingAsRatio("bridge_skin_density_3"));
        }
    }

    double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
    const bool monotonic = mesh.getSettingBoolean("skin_monotonic");
    processSkinPrintFeature(storage, gcode_layer, mesh, mesh_config, extruder_nr, skin_part.skin_fill, *skin_config, pattern, skin_angle, skin_overlap, skin_density, monotonic, added_something);
}

void FffGcodeWriter::processSkinPrintFeature(const SliceDataStorage& storage, LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const PathConfigStorage::MeshPathConfigs& mesh_config, const size_t extruder_nr, const Polygons& area, const GCodePathConfig& config, EFillMethod pattern, const AngleDegrees skin_angle, const coord_t skin_overlap, const Ratio skin_density, const bool monotonic, bool& added_something) const
{
    Polygons skin_polygons;
    Polygons skin_lines;
    std::vector<VariableWidthLines> skin_paths;

    constexpr int infill_multiplier = 1;
    constexpr int extra_infill_shift = 0;
    const size_t wall_line_count = mesh.getSettingAsCount("skin_outline_count"); //Extra Skin Wall Count

    const bool connect_polygons = mesh.getSettingBoolean("connect_skin_polygons");
    coord_t max_resolution = mesh.getSettingInMicrons("meshfix_maximum_resolution");
    coord_t max_deviation = mesh.getSettingInMicrons("meshfix_maximum_deviation");

    constexpr coord_t offset_from_inner_skin_infill = 0;
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const Point infill_origin;
    const bool skip_line_stitching = monotonic;
    constexpr bool connected_zigzags = false;
    constexpr bool use_endpieces = true;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr bool apply_pockets_alternatingly = false;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = mesh.getSettingInMicrons("meshfix_maximum_resolution");

    Infill infill_comp(
        pattern, zig_zaggify_infill, connect_polygons, area, config.getLineWidth(), config.getLineWidth() / skin_density, skin_overlap, infill_multiplier, skin_angle, gcode_layer.z, extra_infill_shift
        , max_resolution, max_deviation
        , wall_line_count, infill_origin,
        connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
    );

    infill_comp.generate(skin_paths, skin_polygons, skin_lines, mesh);

    // add paths
    if (skin_polygons.size() > 0 || skin_lines.size() > 0 || skin_paths.size() > 0)
    {
        added_something = true;
        setExtruder_addPrime(storage, gcode_layer, extruder_nr);
        gcode_layer.setIsInside(true); // going to print stuff inside print object

        if (!skin_paths.empty())
        {
            const size_t skin_extruder_nr = mesh.getSettingAsExtruderNr("roofing_extruder_nr");
            if (extruder_nr == skin_extruder_nr)
            {
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                const ZSeamConfig z_seam_config(mesh.getSettingAsZSeamType("z_seam_type"), mesh.getZSeamHint(), mesh.getSettingAsZSeamCornerPrefType("z_seam_corner"), config.getLineWidth() * 2);
                InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, mesh, extruder_nr,
                    mesh_config.top_skin_config, mesh_config.top_skin_config, mesh_config.top_skin_config, mesh_config.top_skin_config,
                    retract_before_outer_wall, wipe_dist, wipe_dist, skin_extruder_nr, skin_extruder_nr, z_seam_config, skin_paths);
                added_something |= wall_orderer.addToLayer();
            }
        }

        if (!skin_polygons.empty())
        {
            constexpr bool force_comb_retract = false;
            gcode_layer.addTravel(skin_polygons[0][0], force_comb_retract);
            gcode_layer.addPolygonsByOptimizer(skin_polygons, config);
        }

        if (monotonic)
        {
            const coord_t exclude_distance = config.getLineWidth() * 0.8;

            const AngleRadians monotonic_direction = AngleRadians(skin_angle);
            constexpr Ratio flow = 1.0_r;
            const coord_t max_adjacent_distance = config.getLineWidth() * 1.1; // Lines are considered adjacent if they are 1 line width apart, with 10% extra play. The monotonic order is enforced if they are adjacent.
            if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC
                || pattern == EFillMethod::CUBICSUBDIV)
            {
                gcode_layer.addLinesMonotonic(area, skin_lines, config, SpaceFillType::Lines, monotonic_direction, max_adjacent_distance, exclude_distance, mesh.getSettingInMicrons("infill_wipe_dist"), flow);
            }
            else
            {
                const SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
                constexpr coord_t wipe_dist = 0;
                gcode_layer.addLinesMonotonic(area, skin_lines, config, space_fill_type, monotonic_direction, max_adjacent_distance, exclude_distance, wipe_dist, flow);
            }
        }
        else
        {
            std::optional<Point> near_start_location;
            const EFillMethod pattern = (gcode_layer.getLayerNr() == 0) ?
                mesh.getSettingAsFillMethod("top_bottom_pattern_0") :
                mesh.getSettingAsFillMethod("top_bottom_pattern");
            if (pattern == EFillMethod::LINES || pattern == EFillMethod::ZIG_ZAG)
            { // update near_start_location to a location which tries to avoid seams in skin
                near_start_location = getSeamAvoidingLocation(area, skin_angle, gcode_layer.getLastPlannedPositionOrStartingPosition());
            }

            constexpr bool enable_travel_optimization = false;
            constexpr float flow = 1.0;
            if (pattern == EFillMethod::GRID || pattern == EFillMethod::LINES || pattern == EFillMethod::TRIANGLES || pattern == EFillMethod::CUBIC || pattern == EFillMethod::TETRAHEDRAL || pattern == EFillMethod::QUARTER_CUBIC || pattern == EFillMethod::CUBICSUBDIV)
            {
                gcode_layer.addLinesByOptimizer(skin_lines, config, SpaceFillType::Lines, enable_travel_optimization, mesh.getSettingInMicrons("infill_wipe_dist"), flow, near_start_location);
            }
            else
            {
                SpaceFillType space_fill_type = (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines;
                constexpr coord_t wipe_dist = 0;
                gcode_layer.addLinesByOptimizer(skin_lines, config, space_fill_type, enable_travel_optimization, wipe_dist, flow, near_start_location);
            }
        }
    }
}

bool FffGcodeWriter::processIroning(const SliceMeshStorage& mesh, const SliceLayer& layer, const GCodePathConfig& line_config, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const bool ironing_enabled = mesh.getSettingBoolean("ironing_enabled");
    const bool ironing_only_highest_layer = mesh.getSettingBoolean("ironing_only_highest_layer");
    if (ironing_enabled && (!ironing_only_highest_layer || mesh.layer_nr_max_filled_layer == gcode_layer.getLayerNr()))
    {
        added_something |= layer.top_surface.ironing(mesh, line_config, gcode_layer);
    }
    return added_something;
}

bool FffGcodeWriter::addSupportToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr) const
{
    bool support_added = false;
    if (!storage.support.generated || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer)
    {
        return support_added;
    }

    const int support_roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const int support_bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    const int support_roof_skin_extruder_nr = getSettingAsIndex("support_roof_skin_extruder_nr");
    const int support_bottom_skin_extruder_nr = getSettingAsIndex("support_bottom_skin_extruder_nr");
    int support_infill_extruder_nr = (gcode_layer.getLayerNr() <= 0)? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");

    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];
    if (support_layer.support_bottom.empty() && support_layer.support_roof.empty() && support_layer.support_infill_parts.empty() && 
		support_layer.support_roof_interface.empty() && support_layer.support_bottom_interface.empty())
    {
        return support_added;
    }

	if (extruder_nr == support_roof_skin_extruder_nr)
	{
		support_added |= addSupportInterfaceRoofsToGCode(storage, gcode_layer);
	}
    if (extruder_nr == support_infill_extruder_nr)
    {
        support_added |= processSupportInfill(storage, gcode_layer);
    }
    if (extruder_nr == support_roof_extruder_nr)
    {
        support_added |= addSupportRoofsToGCode(storage, gcode_layer);
    }
    if (extruder_nr == support_bottom_extruder_nr)
    {
        support_added |= addSupportBottomsToGCode(storage, gcode_layer);
    }
    if (extruder_nr == support_bottom_skin_extruder_nr)
    {
		support_added |= addSupportInterfaceBottomsToGCode(storage, gcode_layer);
    }
    return support_added;
}

bool FffGcodeWriter::processSupportInfill(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    bool added_something = false;
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())]; // account for negative layer numbers for raft filler layers

    if (gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer || support_layer.support_infill_parts.empty())
    {
        return added_something;
    }

    const int extruder_nr = (gcode_layer.getLayerNr() <= 0) ? getSettingAsIndex("support_extruder_nr_layer_0") : getSettingAsIndex("support_infill_extruder_nr");
    const ExtruderTrain& infill_extruder = *storage.meshgroup->getExtruderTrain(extruder_nr);

    coord_t default_support_line_distance = infill_extruder.getSettingInMicrons("support_line_distance");

    if (gcode_layer.getLayerNr() <= 0)
    {
        default_support_line_distance = infill_extruder.getSettingInMicrons("support_initial_layer_line_distance");
    }

    const int default_support_infill_overlap = infill_extruder.getSettingInMicrons("infill_overlap_mm");
    //const double support_infill_angle = 0;
    // Helper to get the support infill angle
    const auto get_support_infill_angle = [](const SupportStorage& support_storage, const int layer_nr)
    {
        if (layer_nr <= 0)
        {
            // handle negative layer numbers
            const size_t divisor = support_storage.support_infill_angles_layer_0.size();
            const size_t index = ((layer_nr % divisor) + divisor) % divisor;
            return support_storage.support_infill_angles_layer_0.at(index);
        }
        return support_storage.support_infill_angles.at(static_cast<size_t>(layer_nr) % support_storage.support_infill_angles.size());
    };

    const double support_infill_angle = get_support_infill_angle(storage.support, gcode_layer.getLayerNr());

    coord_t default_support_line_width = infill_extruder.getSettingInMicrons("support_line_width");
    if (gcode_layer.getLayerNr() == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
    {
        default_support_line_width *= infill_extruder.getSettingAsRatio("initial_layer_line_width_factor");
    }

    EFillMethod support_pattern = infill_extruder.getSettingAsFillMethod("support_pattern");
    if (gcode_layer.getLayerNr() <= 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG))
    {
        support_pattern = EFillMethod::GRID;
    }
    const bool zig_zaggify_infill = infill_extruder.getSettingBoolean("zig_zaggify_support");
    const bool skip_some_zags = infill_extruder.getSettingBoolean("support_skip_some_zags");
    const int zag_skip_count = infill_extruder.getSettingAsCount("support_zag_skip_count");
    constexpr size_t infill_multiplier = 1; // there is no frontend setting for this (yet)
    constexpr bool use_endpieces = true;
    constexpr coord_t pocket_size = 0;
    constexpr bool connect_polygons = false; // polygons are too distant to connect for sparse support
    const size_t wall_line_count = 0;
    // create a list of outlines and use PathOrderOptimizer to optimize the travel move
    PathOrderOptimizer<const SupportInfillPart*> island_order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition());
    for (const SupportInfillPart& part : support_layer.support_infill_parts)
    {
        island_order_optimizer.addPolygon(&part);
    }
    island_order_optimizer.optimize();

    // Helper to determine the appropriate support area
    const auto get_support_area = [](const Polygons& area, const int layer_nr, const EFillMethod pattern,
        const coord_t line_width, const coord_t brim_line_count)
    {
        if (layer_nr == 0 && pattern == EFillMethod::CONCENTRIC)
        {
            return  area.offset(static_cast<int>(line_width * brim_line_count / 1000));
        }
        return area;
    };

    const auto support_brim_line_count = 0;
    const auto support_connect_zigzags = true;
    const Point infill_origin;
    const coord_t max_resolution = infill_extruder.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_deviation = infill_extruder.getSettingInMicrons("meshfix_maximum_deviation");

    const SupportStructure support_structure = infill_extruder.getSettingAsSupportStructure("support_structure");

    //Print the thicker infill lines first. (double or more layer thickness, infill combined with previous layers)
    const std::vector<SupportInfillPart>& part_list = support_layer.support_infill_parts;
    for (const PathOrderPath<const SupportInfillPart*>& path : island_order_optimizer.paths)
    {
        const SupportInfillPart& part = *path.vertices;
        //always process the wall overlap if walls are generated
        const int current_support_infill_overlap = (part.inset_count_to_generate > 0) ? default_support_infill_overlap : 0;
        //The support infill walls were generated separately, first. Always add them, regardless of how many densities we have.
        std::vector<VariableWidthLines> wall_toolpaths = part.wall_toolpaths;

        if (!wall_toolpaths.empty())
        {
            const GCodePathConfig& config = gcode_layer.configs_storage.support_infill_config[0];
            constexpr bool retract_before_outer_wall = false;
            constexpr coord_t wipe_dist = 0;
            const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
            InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, infill_extruder, extruder_nr,
                config, config, config, config,
                retract_before_outer_wall, wipe_dist, wipe_dist, extruder_nr, extruder_nr, z_seam_config, wall_toolpaths);
            added_something |= wall_orderer.addToLayer();
        }
        if ((default_support_line_distance <= 0 && support_structure != SupportStructure::TREE) || part.infill_area_per_combine_per_density.empty())
        {
            continue;
        }
        for (unsigned int combine_idx = 0; combine_idx < part.infill_area_per_combine_per_density[0].size(); ++combine_idx)
        {
            const coord_t support_line_width = default_support_line_width * (combine_idx + 1);

            Polygons support_polygons;
            std::vector<VariableWidthLines> wall_toolpaths_here;
            Polygons support_lines;
            const size_t max_density_idx = part.infill_area_per_combine_per_density.size() - 1;
            for (size_t density_idx = max_density_idx; (density_idx + 1) > 0; --density_idx)
            {
                if (combine_idx >= part.infill_area_per_combine_per_density[density_idx].size())
                {
                    continue;
                }

                const unsigned int density_factor = 2 << density_idx; // == pow(2, density_idx + 1)
                int support_line_distance_here = default_support_line_distance * density_factor; // the highest density infill combines with the next to create a grid with density_factor 1
                const int support_shift = support_line_distance_here / 2;
                if (density_idx == max_density_idx || support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D)
                {
                    support_line_distance_here /= 2;
                }

                const Polygons& area = get_support_area(part.infill_area_per_combine_per_density[density_idx][combine_idx],
                    gcode_layer.getLayerNr(), support_pattern, support_line_width,
                    support_brim_line_count);
                constexpr size_t wall_count = 0; // Walls are generated somewhere else, so their layers aren't vertically combined.
                constexpr bool skip_stitching = false;
                Infill infill_comp(support_pattern, zig_zaggify_infill, connect_polygons, area,
                    support_line_width, support_line_distance_here, current_support_infill_overlap - (density_idx == max_density_idx ? 0 : wall_line_count * support_line_width),
                    infill_multiplier, support_infill_angle, gcode_layer.z, support_shift,
                    max_resolution, max_deviation,
                    wall_count, infill_origin, skip_stitching, support_connect_zigzags,
                    use_endpieces, skip_some_zags, zag_skip_count, pocket_size);
                infill_comp.generate(wall_toolpaths_here, support_polygons, support_lines, infill_extruder, storage.support.cross_fill_provider);
            }

            setExtruder_addPrime(storage, gcode_layer, extruder_nr); // only switch extruder if we're sure we're going to switch
            gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support

            const bool alternate_inset_direction = infill_extruder.getSettingBoolean("material_alternate_walls");
            const bool alternate_layer_print_direction = alternate_inset_direction && gcode_layer.getLayerNr() % 2 == 1;

            if (!support_polygons.empty())
            {
                constexpr bool force_comb_retract = false;
                gcode_layer.addTravel(support_polygons[0][0], force_comb_retract);
                const ZSeamConfig& z_seam_config = ZSeamConfig();
                constexpr coord_t wall_0_wipe_dist = 0;
                constexpr bool spiralize = false;
                constexpr Ratio flow_ratio = 1.0_r;
                constexpr bool always_retract = false;
                const std::optional<Point> start_near_location = std::optional<Point>();

                gcode_layer.addPolygonsByOptimizer
                (
                    support_polygons,
                    gcode_layer.configs_storage.support_infill_config[combine_idx],
                    z_seam_config,
                    wall_0_wipe_dist,
                    spiralize,
                    flow_ratio,
                    always_retract,
                    alternate_layer_print_direction,
                    start_near_location
                );
                added_something = true;
            }
            if (!support_lines.empty())
            {
                constexpr bool enable_travel_optimization = false;
                constexpr coord_t wipe_dist = 0;
                constexpr Ratio flow_ratio = 1.0;
                const std::optional<Point> near_start_location = std::optional<Point>();
                constexpr double fan_speed = GCodePathConfig::FAN_SPEED_DEFAULT;
                gcode_layer.addLinesByOptimizer
                (
                    support_lines,
                    gcode_layer.configs_storage.support_infill_config[combine_idx],
                    (support_pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines,
                    enable_travel_optimization,
                    wipe_dist,
                    flow_ratio,
                    near_start_location,
                    fan_speed,
                    alternate_layer_print_direction
                );
                added_something = true;
            }
            //If we're printing with a support wall, that support wall generates gap filling as well.
            //If not, the pattern may still generate gap filling (if it's connected infill or zigzag). We still want to print those.
            if (wall_line_count == 0 && !wall_toolpaths_here.empty())
            {
                const GCodePathConfig& config = gcode_layer.configs_storage.support_infill_config[0];
                constexpr bool retract_before_outer_wall = false;
                constexpr coord_t wipe_dist = 0;
                constexpr coord_t simplify_curvature = 0;
                const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, simplify_curvature);
                InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, infill_extruder, extruder_nr,
                    config, config, config, config,
                    retract_before_outer_wall, wipe_dist, wipe_dist, extruder_nr, extruder_nr, z_seam_config, wall_toolpaths_here);
                added_something |= wall_orderer.addToLayer();
            }
        }
    }
    return added_something;
}

bool FffGcodeWriter::addSupportRoofsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

    if (!storage.support.generated 
        || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer 
        || support_layer.support_roof.empty())
    {
        return false; //No need to generate support roof if there's no support.
    }

    const int roof_extruder_nr = getSettingAsIndex("support_roof_extruder_nr");
    const ExtruderTrain& roof_extr = *storage.meshgroup->getExtruderTrain(roof_extruder_nr);

    const EFillMethod pattern = roof_extr.getSettingAsFillMethod("support_roof_pattern");
    double fill_angle = 0;
    if (!storage.support.support_roof_angles.empty())
    {
        // handle negative layer numbers
        int divisor = static_cast<int>(storage.support.support_roof_angles.size());
        int index = ((gcode_layer.getLayerNr() % divisor) + divisor) % divisor;
        fill_angle = storage.support.support_roof_angles.at(index);
    }
    //const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_roof_height", gcode_layer.getLayerNr());
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = false; // connections might happen in mid air in between the infill lines
    constexpr int support_roof_overlap = 0; // the roofs should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    const size_t wall_line_count = roof_extr.getSettingAsCount("support_roof_wall_count");
    constexpr int outline_offset =  0;
    constexpr int extra_infill_shift = 0;
    const Point infill_origin;
    constexpr bool skip_stitching = false;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr bool apply_pockets_alternatingly = false;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = roof_extr.getSettingInMicrons("meshfix_maximum_resolution");

    coord_t support_roof_line_distance = roof_extr.getSettingInMicrons("support_roof_line_distance");
    const coord_t support_roof_line_width = roof_extr.getSettingInMicrons("support_roof_line_width");
    if (gcode_layer.getLayerNr() == 0 && support_roof_line_distance < 2 * support_roof_line_width)
    { // if roof is dense
        support_roof_line_distance *= roof_extr.getSettingAsRatio("initial_layer_line_width_factor");
    }

    const coord_t max_resolution = roof_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_deviation = roof_extr.getSettingInMicrons("meshfix_maximum_deviation");

    Polygons infill_outline = support_layer.support_roof;
    Polygons wall;
    // make sure there is a wall if this is on the first layer
    if (gcode_layer.getLayerNr() == 0)
    {
        wall = support_layer.support_roof.offset(-support_roof_line_width / 2);
        infill_outline = wall.offset(-support_roof_line_width / 2);
    }

    Infill roof_computation(
        pattern, zig_zaggify_infill, connect_polygons, infill_outline, gcode_layer.configs_storage.support_roof_config.getLineWidth(),
        support_roof_line_distance, support_roof_overlap, infill_multiplier, fill_angle, gcode_layer.z, extra_infill_shift,
        max_resolution, max_deviation,
        wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
    );
    Polygons roof_polygons;
    std::vector<VariableWidthLines> roof_paths;
    Polygons roof_lines;
    roof_computation.generate(roof_paths, roof_polygons, roof_lines, roof_extr);
    if ((gcode_layer.getLayerNr() == 0 && wall.empty()) || (gcode_layer.getLayerNr() > 0 && roof_paths.empty() && roof_polygons.empty() && roof_lines.empty()))
    {
        return false; //We didn't create any support roof.
    }
    setExtruder_addPrime(storage, gcode_layer, roof_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
    if (gcode_layer.getLayerNr() == 0)
    {
        gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.support_roof_config);
    }
    if (!roof_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(roof_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(roof_polygons, gcode_layer.configs_storage.support_roof_config);
    }
    if (!roof_paths.empty())
    {
        const GCodePathConfig& config = gcode_layer.configs_storage.support_roof_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);

        InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, roof_extr, roof_extruder_nr,
            config, config, config, config,
            retract_before_outer_wall, wipe_dist, wipe_dist, roof_extruder_nr, roof_extruder_nr, z_seam_config, roof_paths);
        wall_orderer.addToLayer();
    }
    gcode_layer.addLinesByOptimizer(roof_lines, gcode_layer.configs_storage.support_roof_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

bool FffGcodeWriter::addSupportInterfaceRoofsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
	const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

	if (!storage.support.generated
		|| gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer
		|| support_layer.support_roof_interface.empty())
	{
		return false; //No need to generate support roof interface if there's no support.
	}

	const int roof_interface_extruder_nr = getSettingAsIndex("support_roof_skin_extruder_nr");
	const ExtruderTrain& roof_interface_extr = *storage.meshgroup->getExtruderTrain(roof_interface_extruder_nr);

	const EFillMethod pattern = roof_interface_extr.getSettingAsFillMethod("support_roof_pattern");
	const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_roof_height", gcode_layer.getLayerNr());
	const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
	constexpr int support_roof_overlap = 0; // the roofs should never be expanded outwards
    const bool connect_polygons = false; // connections might happen in mid air in between the infill lines
	constexpr int outline_offset = 0;
    constexpr size_t infill_multiplier = 1;
    const size_t wall_line_count = roof_interface_extr.getSettingAsCount("support_roof_wall_count");
	constexpr int extra_infill_shift = 0;
	const Point infill_origin;
    constexpr bool skip_stitching = false;
	constexpr Polygons* perimeter_gaps = nullptr;
	constexpr bool use_endpieces = true;
	constexpr bool connected_zigzags = false;
	constexpr bool skip_some_zags = false;
	constexpr int zag_skip_count = 0;
	constexpr bool apply_pockets_alternatingly = false;
	constexpr coord_t pocket_size = 0;
	const coord_t maximum_resolution = roof_interface_extr.getSettingInMicrons("meshfix_maximum_resolution");

	coord_t support_roof_line_distance = roof_interface_extr.getSettingInMicrons("support_roof_line_distance");
	const coord_t support_roof_line_width = roof_interface_extr.getSettingInMicrons("support_roof_line_width");
    const coord_t max_resolution = roof_interface_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_deviation = roof_interface_extr.getSettingInMicrons("meshfix_maximum_deviation");
	if (gcode_layer.getLayerNr() == 0 && support_roof_line_distance < 2 * support_roof_line_width)
	{ // if roof is dense
		support_roof_line_distance *= roof_interface_extr.getSettingAsRatio("initial_layer_line_width_factor");
	}

	Polygons infill_outline = support_layer.support_roof_interface;
	Polygons wall;
	// make sure there is a wall if this is on the first layer
	if (gcode_layer.getLayerNr() == 0)
	{
		wall = support_layer.support_roof_interface.offset(-support_roof_line_width / 2);
		infill_outline = wall.offset(-support_roof_line_width / 2);
	}

	const coord_t z_distance_top = getSettingInMicrons("support_top_distance");

    Infill roof_computation(
        pattern, zig_zaggify_infill, connect_polygons, infill_outline, gcode_layer.configs_storage.support_roof_interface_config.getLineWidth(),
        support_roof_line_distance, support_roof_overlap, infill_multiplier, fill_angle, gcode_layer.z - z_distance_top, extra_infill_shift,
        max_resolution, max_deviation,
        wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
    );

	Polygons roof_polygons;
    std::vector<VariableWidthLines> roof_paths;
	Polygons roof_lines;
	roof_computation.generate(roof_paths, roof_polygons, roof_lines, roof_interface_extr);
	if ((gcode_layer.getLayerNr() == 0 && wall.empty()) || (roof_paths.empty() && roof_polygons.empty() && roof_lines.empty()))
	{
		return false; //We didn't create any support roof.
	}
	setExtruder_addPrime(storage, gcode_layer, roof_interface_extruder_nr);
	gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
	if (gcode_layer.getLayerNr() == 0)
	{
		gcode_layer.addPolygonsByOptimizer(wall, gcode_layer.configs_storage.support_roof_interface_config);
	}
	if (!roof_polygons.empty())
	{
		constexpr bool force_comb_retract = false;
		gcode_layer.addTravel(roof_polygons[0][0], force_comb_retract);
		gcode_layer.addPolygonsByOptimizer(roof_polygons, gcode_layer.configs_storage.support_roof_interface_config);
	}

    if (!roof_paths.empty())
    {
        const GCodePathConfig& config = gcode_layer.configs_storage.support_roof_interface_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
        InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, roof_interface_extr, roof_interface_extruder_nr,
            config, config, config, config,
            retract_before_outer_wall, wipe_dist, wipe_dist, roof_interface_extruder_nr, roof_interface_extruder_nr, z_seam_config, roof_paths);
        wall_orderer.addToLayer();
    }
	gcode_layer.addLinesByOptimizer(roof_lines, gcode_layer.configs_storage.support_roof_interface_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
	return true;
}

bool FffGcodeWriter::addSupportBottomsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
    const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

    if (!storage.support.generated 
        || gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer 
        || support_layer.support_bottom.empty())
    {
        return false; //No need to generate support bottoms if there's no support.
    }

    const int bottom_extruder_nr = getSettingAsIndex("support_bottom_extruder_nr");
    const ExtruderTrain& bottom_extr = *storage.meshgroup->getExtruderTrain(bottom_extruder_nr);

    const EFillMethod pattern = bottom_extr.getSettingAsFillMethod("support_bottom_pattern");
    //const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_bottom_height", gcode_layer.getLayerNr());
    double fill_angle = 0;
    if (!storage.support.support_bottom_angles.empty())
    {
        // handle negative layer numbers
        int divisor = static_cast<int>(storage.support.support_bottom_angles.size());
        int index = ((gcode_layer.getLayerNr() % divisor) + divisor) % divisor;
        fill_angle = storage.support.support_bottom_angles.at(index);
    }
    const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = true; // less retractions and less moves only make the bottoms easier to print
    constexpr int support_bottom_overlap = 0; // the bottoms should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
    constexpr int outline_offset =  0;
    constexpr int extra_infill_shift = 0;
    const size_t wall_line_count = bottom_extr.getSettingAsCount("support_bottom_wall_count");
    const Point infill_origin;
    constexpr bool skip_stitching = false;
    constexpr Polygons* perimeter_gaps = nullptr;
    constexpr bool use_endpieces = true;
    constexpr bool connected_zigzags = false;
    constexpr bool skip_some_zags = false;
    constexpr int zag_skip_count = 0;
    constexpr bool apply_pockets_alternatingly = false;
    constexpr coord_t pocket_size = 0;
    const coord_t maximum_resolution = bottom_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_resolution = bottom_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_deviation = bottom_extr.getSettingInMicrons("meshfix_maximum_deviation");
    const coord_t support_bottom_line_distance = bottom_extr.getSettingInMicrons("support_bottom_line_distance"); // note: no need to apply initial line width factor; support bottoms cannot exist on the first layer
    Infill bottom_computation(
        pattern, zig_zaggify_infill, connect_polygons, support_layer.support_bottom, gcode_layer.configs_storage.support_bottom_config.getLineWidth(),
        support_bottom_line_distance, support_bottom_overlap, infill_multiplier, fill_angle, gcode_layer.z, extra_infill_shift,
        max_resolution, max_deviation,
        wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
    );

    Polygons bottom_polygons;
    std::vector<VariableWidthLines> bottom_paths;
    Polygons bottom_lines;

    bottom_computation.generate(bottom_paths, bottom_polygons, bottom_lines, bottom_extr);
    if (bottom_paths.empty() && bottom_polygons.empty() && bottom_lines.empty())
    {
        return false;
    }
    setExtruder_addPrime(storage, gcode_layer, bottom_extruder_nr);
    gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support

    if (!bottom_polygons.empty())
    {
        constexpr bool force_comb_retract = false;
        gcode_layer.addTravel(bottom_polygons[0][0], force_comb_retract);
        gcode_layer.addPolygonsByOptimizer(bottom_polygons, gcode_layer.configs_storage.support_bottom_config);
    }

    if (!bottom_paths.empty())
    {
        const GCodePathConfig& config = gcode_layer.configs_storage.support_bottom_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
        InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, bottom_extr, bottom_extruder_nr,
            config, config, config, config,
            retract_before_outer_wall, wipe_dist, wipe_dist, bottom_extruder_nr, bottom_extruder_nr, z_seam_config, bottom_paths);
        wall_orderer.addToLayer();
    }
    gcode_layer.addLinesByOptimizer(bottom_lines, gcode_layer.configs_storage.support_bottom_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

bool FffGcodeWriter::addSupportInterfaceBottomsToGCode(const SliceDataStorage& storage, LayerPlan& gcode_layer) const
{
	const SupportLayer& support_layer = storage.support.supportLayers[std::max(0, gcode_layer.getLayerNr())];

	if (!storage.support.generated
		|| gcode_layer.getLayerNr() > storage.support.layer_nr_max_filled_layer
		|| support_layer.support_bottom_interface.empty())
	{
		return false;
	}

	const int bottom_skin_extruder_nr = getSettingAsIndex("support_bottom_skin_extruder_nr");
	const ExtruderTrain& bottom_interface_extr = *storage.meshgroup->getExtruderTrain(bottom_skin_extruder_nr);

	const EFillMethod pattern = bottom_interface_extr.getSettingAsFillMethod("support_bottom_pattern");
	const double fill_angle = supportInterfaceFillAngle(storage, pattern, "support_bottom_height", gcode_layer.getLayerNr());
	const bool zig_zaggify_infill = pattern == EFillMethod::ZIG_ZAG;
    const bool connect_polygons = true; // less retractions and less moves only make the bottoms easier to print
	constexpr int support_bottom_overlap = 0; // the bottoms should never be expanded outwards
    constexpr size_t infill_multiplier = 1;
	constexpr int outline_offset = 0;
	constexpr int extra_infill_shift = 0;
    const size_t wall_line_count = bottom_interface_extr.getSettingAsCount("support_bottom_wall_count");
	const Point infill_origin;
    constexpr bool skip_stitching = false;
	constexpr Polygons* perimeter_gaps = nullptr;
	constexpr bool use_endpieces = true;
	constexpr bool connected_zigzags = false;
	constexpr bool skip_some_zags = false;
	constexpr int zag_skip_count = 0;
	constexpr bool apply_pockets_alternatingly = false;
	constexpr coord_t pocket_size = 0;
	const coord_t maximum_resolution = bottom_interface_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_resolution = bottom_interface_extr.getSettingInMicrons("meshfix_maximum_resolution");
    const coord_t max_deviation = bottom_interface_extr.getSettingInMicrons("meshfix_maximum_deviation");
	const coord_t support_bottom_line_distance = bottom_interface_extr.getSettingInMicrons("support_bottom_line_distance"); // note: no need to apply initial line width factor; support bottoms cannot exist on the first layer

    Infill bottom_interface_computation(
		pattern, zig_zaggify_infill, connect_polygons, support_layer.support_bottom_interface, gcode_layer.configs_storage.support_bottom_interface_config.getLineWidth(),
		support_bottom_line_distance, support_bottom_overlap, infill_multiplier, fill_angle, gcode_layer.z, extra_infill_shift,
        max_resolution, max_deviation,
        wall_line_count, infill_origin, skip_stitching, connected_zigzags, use_endpieces, skip_some_zags, zag_skip_count, pocket_size
	);
	Polygons bottom_interface_polygons;
    std::vector<VariableWidthLines> bottom_interface_paths;
	Polygons bottom_interface_lines;
	bottom_interface_computation.generate(bottom_interface_paths, bottom_interface_polygons, bottom_interface_lines, bottom_interface_extr);
	if (bottom_interface_paths.empty() && bottom_interface_polygons.empty() && bottom_interface_lines.empty())
	{
		return false;
	}
	setExtruder_addPrime(storage, gcode_layer, bottom_skin_extruder_nr);
	gcode_layer.setIsInside(false); // going to print stuff outside print object, i.e. support
	
    if (!bottom_interface_polygons.empty())
	{
		constexpr bool force_comb_retract = false;
		gcode_layer.addTravel(bottom_interface_polygons[0][0], force_comb_retract);
		gcode_layer.addPolygonsByOptimizer(bottom_interface_polygons, gcode_layer.configs_storage.support_bottom_interface_config);
	}
    if (!bottom_interface_paths.empty())
    {
        const GCodePathConfig& config = gcode_layer.configs_storage.support_bottom_interface_config;
        constexpr bool retract_before_outer_wall = false;
        constexpr coord_t wipe_dist = 0;
        const ZSeamConfig z_seam_config(EZSeamType::SHORTEST, gcode_layer.getLastPlannedPositionOrStartingPosition(), EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE, false);
        InsetOrderOptimizer wall_orderer(*this, storage, gcode_layer, bottom_interface_extr, bottom_skin_extruder_nr,
                                         config, config, config, config, 
                                         retract_before_outer_wall, wipe_dist, wipe_dist, bottom_skin_extruder_nr, bottom_skin_extruder_nr, z_seam_config, bottom_interface_paths);
        wall_orderer.addToLayer();
    }
	gcode_layer.addLinesByOptimizer(bottom_interface_lines, gcode_layer.configs_storage.support_bottom_interface_config, (pattern == EFillMethod::ZIG_ZAG) ? SpaceFillType::PolyLines : SpaceFillType::Lines);
    return true;
}

double FffGcodeWriter::supportInterfaceFillAngle(const SliceDataStorage& storage, const EFillMethod pattern, const std::string interface_height_setting, const int layer_nr) const
{
    if (pattern == EFillMethod::CONCENTRIC)
    {
        return 0; //Concentric has no rotation.
    }
    if (pattern == EFillMethod::TRIANGLES)
    {
        return 90; //Triangular support interface shouldn't alternate every layer.
    }

    for (const SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingInMicrons(interface_height_setting) >= 2 * getSettingInMicrons("layer_height"))
        {
            //Some roofs are quite thick.
            //Alternate between the two kinds of diagonal: / and \ .
            // + 2) % 2 is to handle negative layer numbers.
            return 45 + (((layer_nr % 2) + 2) % 2) * 90;
        }
    }

    return 90; //Perpendicular to support lines.
}

void FffGcodeWriter::setExtruder_addPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr) const //要加入extruder_order的序列，可以判断出这个位次，知道每个喷头的primetower是否已经planned，规划出primetower的打印顺序和逻辑
{
	if (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED)
		setExtruder_addNestedPrime(storage, gcode_layer, extruder_nr);
	else
		setExtruder_addInterlacedPrime(storage, gcode_layer, extruder_nr);
}

void FffGcodeWriter::setExtruder_addNestedPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr) const
{
	if (extruder_nr == -1) // an object with extruder_nr==-1 means it will be printed with any current nozzle
	{
		return;
	}

	const unsigned int outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];

	const unsigned int last_planned_extruder = gcode_layer.getExtruder(); //Get the last planned extruder
	
	if (last_planned_extruder == static_cast<unsigned int>(extruder_nr) && !(static_cast<unsigned int>(extruder_nr) == outermost_prime_tower_extruder && gcode_layer.getLayerNr() >= -Raft::getFillerLayerCount(storage))) //No unnecessary switches, unless switching to extruder for the outer shell of the prime tower.
	{
		return;
	}

	if (gcode_layer.getPrimeTowerIsPlanned(extruder_nr))	return;

	bool extruder_changed = gcode_layer.setExtruder(extruder_nr);

	if (extruder_changed)
	{
		if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
		{
			ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);

			if (train->getSettingBoolean("prime_blob_enable"))
			{
				// We always prime an extruder, but whether it will be a prime blob/poop depends on if prime blob is enabled.
				// This is decided in GCodeExport::writePrimeTrain().
				bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
				Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
				gcode_layer.addTravel(prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos);
				gcode_layer.planPrime();
			}
		}

		if (gcode_layer.getLayerNr() == 0 && !gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
		{
			processSkirtBrim(storage, gcode_layer, extruder_nr);
		}
	}

	// The first layer of the prime tower is printed with one material only, so do not prime another material on the
	// first layer again.
    if (((extruder_changed && gcode_layer.getLayerNr() > 0) || static_cast<unsigned int>(extruder_nr) == outermost_prime_tower_extruder) && gcode_layer.getLayerNr() >= -Raft::getFillerLayerCount(storage)) //Always print a prime tower with outermost extruder.
    {
		addPrimeTower(storage, gcode_layer, last_planned_extruder, extruder_changed);
	}
}

void FffGcodeWriter::setExtruder_addInterlacedPrime(const SliceDataStorage& storage, LayerPlan& gcode_layer, int extruder_nr)const
{
	if (extruder_nr == -1) // an object with extruder_nr==-1 means it will be printed with any current nozzle
		return;

	int previous_extruder = gcode_layer.getExtruder();
	if (previous_extruder == extruder_nr) { return; }
	bool extruder_changed = gcode_layer.setExtruder(extruder_nr);

	if (extruder_changed)
	{
		if (extruder_prime_layer_nr[extruder_nr] == gcode_layer.getLayerNr())
		{
			ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);

			if (train->getSettingBoolean("prime_blob_enable"))
			{ // only move to prime position if we do a blob/poop
			  // ideally the prime position would be respected whether we do a blob or not,
			  // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
			  // which is needed to automatically ignore the prime position for the UM3 machine when blob is disabled
				bool prime_pos_is_abs = train->getSettingBoolean("extruder_prime_pos_abs");
				Point prime_pos = Point(train->getSettingInMicrons("extruder_prime_pos_x"), train->getSettingInMicrons("extruder_prime_pos_y"));
				gcode_layer.addTravel(prime_pos_is_abs ? prime_pos : gcode_layer.getLastPlannedPositionOrStartingPosition() + prime_pos);

				gcode_layer.planPrime();
			}
		}

		if (gcode_layer.getLayerNr() == 0 && !gcode_layer.getSkirtBrimIsPlanned(extruder_nr))
		{
			processSkirtBrim(storage, gcode_layer, extruder_nr);
		}
		if (gcode_layer.getLayerNr() >= -Raft::getFillerLayerCount(storage))
		{
			addPrimeTower(storage, gcode_layer, previous_extruder);
		}
	}
}

void FffGcodeWriter::addPrimeTower(const SliceDataStorage& storage, LayerPlan& gcode_layer, int prev_extruder, bool extruder_changed) const
{
    if (!getSettingBoolean("prime_tower_enable"))	return;

	if (gcode_layer.getLayerNr() == 0 && getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED && storage.primeTower.extruder_order[0] != gcode_layer.getExtruder())
	{
		return;
	}

	if (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED)
	{
		const int outermost_prime_tower_extruder = storage.primeTower.extruder_order[0];
		if (gcode_layer.getLayerNr() == 0 && getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT 
			&& static_cast<int>(gcode_layer.getExtruder())!= outermost_prime_tower_extruder && (!storage.support.supportLayers[0].support_infill_parts.empty() || !storage.support.supportLayers[0].support_bottom.empty() || !storage.support.supportLayers[0].support_roof.empty()))
		{
			return;
		}

		bool allFlag = false;

		const std::vector<unsigned int>& extruder_order = (gcode_layer.getLayerNr() < 0) ? extruder_order_per_layer_negative_layers[extruder_order_per_layer_negative_layers.size() + gcode_layer.getLayerNr()]
			: extruder_order_per_layer[gcode_layer.getLayerNr()];


        if (gcode_layer.getLayerNr() >= -Raft::getFillerLayerCount(storage) && extruder_order.size() == 1)
            allFlag = true;

		if(extruder_changed && gcode_layer.getExtruder() == outermost_prime_tower_extruder)
			allFlag = true;

		storage.primeTower.addToGcode(storage, gcode_layer, gcode, prev_extruder, gcode_layer.getExtruder(), allFlag);
	}
	else
	{
		storage.interlacedprimeTower.addToGcode(storage, gcode_layer, gcode, prev_extruder, gcode_layer.getExtruder());
	}
}

bool FffGcodeWriter::partitionInfillBySkinAbove(Polygons& infill_below_skin, Polygons& infill_not_below_skin, const LayerPlan& gcode_layer, const SliceMeshStorage& mesh, const SliceLayerPart& part, coord_t infill_line_width)
{
    constexpr coord_t tiny_infill_offset = 20;
    const auto skin_edge_support_layers = mesh.getSettingAsCount("skin_edge_support_layers");
    Polygons skin_above_combined;  // skin regions on the layers above combined with small gaps between

    // working from the highest layer downwards, combine the regions of skin on all the layers
    // but don't let the regions merge together
    // otherwise "terraced" skin regions on separate layers will look like a single region of unbroken skin

    for (size_t i = skin_edge_support_layers; i > 0; --i)
    {
        const size_t skin_layer_nr = gcode_layer.getLayerNr() + i;
        if (skin_layer_nr < mesh.layers.size())
        {
            for (const SliceLayerPart& part : mesh.layers[skin_layer_nr].parts)
            {
                for (const SkinPart& skin_part : part.skin_parts)
                {
                    if (!skin_above_combined.empty())
                    {
                        // does this skin part overlap with any of the skin parts on the layers above?
                        const Polygons overlap = skin_above_combined.intersection(skin_part.outline);
                        if (!overlap.empty())
                        {
                            // yes, it overlaps, need to leave a gap between this skin part and the others
                            if (i > 1) // this layer is the 2nd or higher layer above the layer whose infill we're printing
                            {
                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //     -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------- ----------

                                // expand the overlap region slightly to make a small gap
                                const Polygons overlap_expanded = overlap.offset(tiny_infill_offset);
                                // subtract the expanded overlap region from the regions accumulated from higher layers
                                skin_above_combined = skin_above_combined.difference(overlap_expanded);
                                // subtract the expanded overlap region from this skin part and add the remainder to the overlap region
                                skin_above_combined.add(skin_part.outline.difference(overlap_expanded));
                                // and add the overlap area as well
                                skin_above_combined.add(overlap);
                            }
                            else // this layer is the 1st layer above the layer whose infill we're printing
                            {
                                // add this layer's skin region without subtracting the overlap but still make a gap between this skin region and what has been accumulated so far
                                // we do this so that these skin region edges will definitely have infill walls below them

                                // looking from the side, if the combined regions so far look like this...
                                //
                                //     ----------------------------------
                                //
                                // and the new skin part looks like this...
                                //
                                //             -------------------------------------
                                //
                                // the result should be like this...
                                //
                                //     ------- -------------------------------------

                                skin_above_combined = skin_above_combined.difference(skin_part.outline.offset(tiny_infill_offset));
                                skin_above_combined.add(skin_part.outline);
                            }
                        }
                        else // no overlap
                        {
                            skin_above_combined.add(skin_part.outline);
                        }
                    }
                    else // this is the first skin region we have looked at
                    {
                        skin_above_combined.add(skin_part.outline);
                    }
                }
            }
        }
        // the shrink/expand here is to remove regions of infill below skin that are narrower than the width of the infill walls otherwise the infill walls could merge and form a bump
        infill_below_skin = skin_above_combined.intersection(part.infill_area_per_combine_per_density.back().front()).offset(-infill_line_width).offset(infill_line_width);

        constexpr bool remove_small_holes_from_infill_below_skin = true;
        constexpr double min_area_multiplier = 25;
        const double min_area = INT2MM(infill_line_width) * INT2MM(infill_line_width) * min_area_multiplier;
        infill_below_skin.removeSmallAreas(min_area, remove_small_holes_from_infill_below_skin);

        // there is infill below skin, is there also infill that isn't below skin?
        infill_not_below_skin = part.infill_area_per_combine_per_density.back().front().difference(infill_below_skin);
        infill_not_below_skin.removeSmallAreas(min_area);
    }

    // need to take skin/infill overlap that was added in SkinInfillAreaComputation::generateInfill() into account
    const coord_t infill_skin_overlap = mesh.getSettingInMicrons((part.wall_toolpaths.size() > 1) ? "wall_line_width_x" : "wall_line_width_0") / 2;
    const Polygons infill_below_skin_overlap = infill_below_skin.offset(-(infill_skin_overlap + tiny_infill_offset));

    return !infill_below_skin_overlap.empty() && !infill_not_below_skin.empty();
}
void FffGcodeWriter::finalize()
{
    double print_time = gcode.getSumTotalPrintTimes();
    std::vector<double> filament_used;
    std::vector<std::string> material_names;
    std::vector<std::string> material_keys;
    std::vector<bool> extruder_is_used;
    std::vector<double>filament_weights;
    for (int extr_nr = 0; extr_nr < getSettingAsCount("machine_extruder_count"); extr_nr++)
    {
        filament_used.emplace_back(gcode.getTotalFilamentUsed(extr_nr));
		material_names.emplace_back(gcode.getMaterialNAME(extr_nr));
        material_keys.emplace_back(gcode.getMaterialKEY(extr_nr));
        extruder_is_used.push_back(gcode.getExtruderIsUsed(extr_nr));

        filament_weights.emplace_back(gcode.getTotalFilamentWeight(extr_nr));
    }
    std::string prefix = gcode.getFileHeader(extruder_is_used, &print_time, filament_used, filament_weights, material_names, material_keys);
    if (CommandSocket::isInstantiated())
    {
        CommandSocket::getInstance()->sendGCodePrefix(prefix);
    }
    else
    {
        log("Gcode header after slicing:\n");
        log(prefix.c_str());
        log("End of gcode header.\n");
    }
    if (getSettingBoolean("acceleration_enabled"))
    {
        gcode.writePrintAcceleration(getSettingInMillimetersPerSecond("machine_acceleration"));
        gcode.writeTravelAcceleration(getSettingInMillimetersPerSecond("machine_acceleration"));
    }
    if (getSettingBoolean("jerk_enabled"))
    {
        gcode.writeJerk(getSettingInMillimetersPerSecond("machine_max_jerk_xy"));
    }
    if (gcode.getCurrentMaxZFeedrate() > 0)
    {
        gcode.writeMaxZFeedrate(getSettingInMillimetersPerSecond("machine_max_feedrate_z"));
    }

    std::string end_gcode = getSettingString("machine_end_gcode");

    if (end_gcode.length() > 0 && getSettingBoolean("relative_extrusion"))
    {
        gcode.writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
    }
    gcode.finalize(end_gcode.c_str());

    // set extrusion mode back to "normal"
    const bool set_relative_extrusion_mode = (gcode.getFlavor() == EGCodeFlavor::REPRAP);
    gcode.writeExtrusionMode(set_relative_extrusion_mode);
    for (int e = 0; e < getSettingAsCount("machine_extruder_count"); e++)
    {
        gcode.writeTemperatureCommand(e, 0, false);
    }
    gcode.writeComment("End of Gcode");

    gcode.updateFileHeaderInfo(extruder_is_used, &print_time, filament_used, filament_weights, material_names, material_keys);
}
}//namespace cura

