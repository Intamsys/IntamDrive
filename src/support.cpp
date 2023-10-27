//Copyright (c) 2017 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include <cmath> // sqrt, round
#include <utility> // pair
#include <deque>
#include <fstream> //ifstream.good() 

#ifdef _OPENMP
#include <omp.h>
#endif // _OPENMP

#include "support.h"
#include "BoostInterface.hpp"
#include "utils/math.h"
#include "progress/Progress.h"
#include "infill.h"
#include "infill/SpaceFillingTreeFill.h"
#include "infill/SierpinskiFillProvider.h"
#include "SkeletalTrapezoidation.h"
#include "utils/VoronoiUtils.h"
#include "utils/Simplify.h"
namespace cura 
{

bool AreaSupport::handleSupportModifierMesh(SliceDataStorage& storage, const SettingsBaseVirtual& mesh, const Slicer* slicer)
{
    if (!mesh.getSettingBoolean("anti_overhang_mesh") && !mesh.getSettingBoolean("support_mesh"))
    {
        return false;
    }
    enum ModifierType { ANTI_OVERHANG, SUPPORT_DROP_DOWN, SUPPORT_VANILLA };
    ModifierType modifier_type = (mesh.getSettingBoolean("anti_overhang_mesh"))? ANTI_OVERHANG : ((mesh.getSettingBoolean("support_mesh_drop_down"))? SUPPORT_DROP_DOWN : SUPPORT_VANILLA);
    for (unsigned int layer_nr = 0; layer_nr < slicer->layers.size(); layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
        const SlicerLayer& slicer_layer = slicer->layers[layer_nr];
        switch (modifier_type)
        {
        case ANTI_OVERHANG:
            support_layer.anti_overhang.add(slicer_layer.polygons);
            break;
        case SUPPORT_DROP_DOWN:
            support_layer.support_mesh_drop_down.add(slicer_layer.polygons);
            break;
        case SUPPORT_VANILLA:
            support_layer.support_mesh.add(slicer_layer.polygons);
            break;
        }
    }
    return true;
}

void AreaSupport::splitGlobalSupportAreasIntoSupportInfillParts(SliceDataStorage& storage, const std::vector<Polygons>& global_support_areas_per_layer, unsigned int total_layer_count)
{
    if (total_layer_count == 0){
        return;
    }

    size_t min_layer = 0;
    size_t max_layer = total_layer_count - 1;

    const ExtruderTrain& infill_extr = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const EFillMethod support_pattern = infill_extr.getSettingAsFillMethod("support_pattern");
    const coord_t support_line_width = infill_extr.getSettingInMicrons("support_line_width");

    // the wall line count is used for calculating insets, and we generate support infill patterns within the insets
	unsigned int wall_line_count = infill_extr.getSettingAsCount("support_wall_count");

    // generate separate support islands
    for (unsigned int layer_nr = 0; layer_nr < total_layer_count - 1; ++layer_nr)
    {
        unsigned int wall_line_count_this_layer = wall_line_count;
        if (layer_nr == 0 && (support_pattern == EFillMethod::LINES || support_pattern == EFillMethod::ZIG_ZAG))
        { // the first layer will be printed wit ha grid pattern
            wall_line_count_this_layer++;
        }
        assert(storage.support.supportLayers[layer_nr].support_infill_parts.empty() && "support infill part list is supposed to be uninitialized");

        const Polygons& global_support_areas = global_support_areas_per_layer[layer_nr];
        if (global_support_areas.size() == 0 || layer_nr < min_layer || layer_nr > max_layer)
        {
            // initialize support_infill_parts empty
            storage.support.supportLayers[layer_nr].support_infill_parts.clear();
            continue;
        }

        std::vector<PolygonsPart> support_islands = global_support_areas.splitIntoParts();
        for (const PolygonsPart& island_outline : support_islands)
        {
            coord_t support_line_width_here = support_line_width;
            if (layer_nr == 0 && storage.getSettingAsPlatformAdhesion("adhesion_type") != EPlatformAdhesion::RAFT)
            {
                support_line_width_here *= infill_extr.getSettingAsRatio("initial_layer_line_width_factor");
            }
            // we don't generate insets and infill area for the parts yet because later the skid/brim and prime
            // tower will remove themselves from the support, so the outlines of the parts can be changed.
            SupportInfillPart support_infill_part(island_outline, support_line_width_here, wall_line_count_this_layer);

            storage.support.supportLayers[layer_nr].support_infill_parts.push_back(support_infill_part);
        }
    }
}

void AreaSupport::generateSupportInfillFeatures(SliceDataStorage& storage)
{
    AreaSupport::generateGradualSupport(storage);

    // combine support infill layers
    AreaSupport::combineSupportInfillLayers(storage);

    AreaSupport::cleanup(storage);
}

void AreaSupport::generateGradualSupport(SliceDataStorage& storage)
{
    //
    // # How gradual support infill works:
    // The gradual support infill uses the same technic as the gradual infill. Here is an illustration
    //
    //          A part of the model |
    //                              V
    //              __________________________       <-- each gradual support infill step consists of 2 actual layers
    //            / |                         |      <-- each step halfs the infill density.
    //          /_ _ _ _ _ _ _ _ _ _ _ _ _ _ _|      <-- infill density 1 x 1/(2^2) (25%)
    //        / |   |                         |      <-- infill density 1 x 1/(2^1) (50%)
    //      /_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _|
    //    / |   |   |                         |      <-- infill density 1 x 1/(2^0) (100%)
    //  /_____________________________________|      <-- the current layer, the layer which we are processing right now.
    //
    //  | 0 | 1 | 2 |              3          |      <-- areas with different densities, where densities (or sparsities) are represented by numbers 0, 1, 2, etc.
    //  more dense    ---->      less dense
    //
    // In the example above, it shows a part of the model. For the gradual support infill, we process each layer in the following way:
    //  -> Separate the generated support areas into isolated islands, and we process those islands one by one
    //     so that the outlines and the gradual infill areas of an island can be printed together.
    //      
    //  -> Calculate a number of layer groups, each group consists of layers that will be infilled with the same density.
    //     The maximum number of densities is defined by the parameter "max graudual support steps". For example, it the <max steps> is 5,
    //     It means that we can have at most 5 different densities, each is 1/2 than the other. So, it will looks like below:
    //           |  step  |    density    |  description              |
    //           |    0   |   1 / (2^0)   |  100% the most dense      |
    //           |    1   |   1 / (2^1)   |                           |
    //           |    2   |   1 / (2^2)   |                           |
    //           |    3   |   1 / (2^3)   |                           |
    //           |    4   |   1 / (2^4)   |                           |
    //           |    5   |   1 / (2^5)   |  3.125% the least dense   |
    //
    //  -> With those layer groups, we can project areas with different densities onto the layer we are currently processing.
    //     Like in the illustration above, you can see 4 different areas with densities ranging from 0 to 3.
    //
    //  -> We save those areas with different densities into the SupportInfillPart data structure, which holds all the support infill
    //     information of a support island on a specific layer.
    //
    //  -> Note that this function only does the above, which is identifying and storing support infill areas with densities.
    //     The actual printing part is done in FffGcodeWriter.
    //
    const unsigned int total_layer_count = storage.print_layer_count;
    const ExtruderTrain& infill_extruder = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const unsigned int gradual_support_step_height = infill_extruder.getSettingInMicrons("gradual_support_infill_step_height");
    const unsigned int max_density_steps = infill_extruder.getSettingAsCount("gradual_support_infill_steps");

    const coord_t wall_count = infill_extruder.getSettingAsCount("support_wall_count");
    const coord_t wall_width = infill_extruder.getSettingInMicrons("support_line_width");
    const coord_t overlap = infill_extruder.getSettingInMicrons("infill_overlap_mm");

    // no early-out for this function; it needs to initialize the [infill_area_per_combine_per_density]
    float layer_skip_count = 8; // skip every so many layers as to ignore small gaps in the model making computation more easy
    size_t gradual_support_step_layer_count = round_divide(gradual_support_step_height, storage.getSettingInMicrons("layer_height")); //The difference in layer count between consecutive density infill areas.

    // make gradual_support_step_height divisable by layer_skip_count
    const float n_skip_steps_per_gradual_step = std::max(1.0f, std::ceil(gradual_support_step_layer_count / layer_skip_count)); // only decrease layer_skip_count to make it a divisor of gradual_support_step_layer_count
    layer_skip_count = gradual_support_step_layer_count / n_skip_steps_per_gradual_step;

    size_t min_layer = 0;
    size_t max_layer = total_layer_count - 1;

    // compute different density areas for each support island
    for (unsigned int layer_nr = 0; layer_nr < total_layer_count - 1; ++layer_nr)
    {
        if (layer_nr < min_layer || layer_nr > max_layer)
        {
            continue;
        }

        // generate separate support islands and calculate density areas for each island
        std::vector<SupportInfillPart>& support_infill_parts = storage.support.supportLayers[layer_nr].support_infill_parts;
        for (unsigned int part_idx = 0; part_idx < support_infill_parts.size(); ++part_idx)
        {
            SupportInfillPart& support_infill_part = support_infill_parts[part_idx];

            Polygons original_area = support_infill_part.getInfillArea();
            if (original_area.empty())
            {
                continue;
            }
            // NOTE: This both generates the walls _and_ returns the _actual_ infill area (the one _without_ walls) for use in the rest of the method.
            const Polygons infill_area = Infill::generateWallToolPaths(support_infill_part.wall_toolpaths, original_area, support_infill_part.inset_count_to_generate, wall_width, 0, infill_extruder);
            const AABB& this_part_boundary_box = support_infill_part.outline_boundary_box;

            // calculate density areas for this island
            Polygons less_dense_support = infill_area; // one step less dense with each density_step
            for (unsigned int density_step = 0; density_step < max_density_steps; ++density_step)
            {
                size_t min_layer = layer_nr + density_step * gradual_support_step_layer_count + layer_skip_count;
                size_t max_layer = layer_nr + (density_step + 1) * gradual_support_step_layer_count;

                for (float upper_layer_idx = min_layer; upper_layer_idx <= max_layer; upper_layer_idx += layer_skip_count)
                {
                    if (static_cast<unsigned int>(upper_layer_idx) >= total_layer_count)
                    {
                        less_dense_support.clear();
                        break;
                    }

                    // compute intersections with relevant upper parts
                    const std::vector<SupportInfillPart> upper_infill_parts = storage.support.supportLayers[upper_layer_idx].support_infill_parts;
                    Polygons relevant_upper_polygons;
                    for (unsigned int upper_part_idx = 0; upper_part_idx < upper_infill_parts.size(); ++upper_part_idx)
                    {
                        if (support_infill_part.outline.empty())
                        {
                            continue;
                        }

                        // we compute intersection based on support infill areas
                        const AABB& upper_part_boundary_box = upper_infill_parts[upper_part_idx].outline_boundary_box;
                        //
                        // Here we are comparing the **outlines** of the infill areas
                        //
                        // legend:
                        //   ^ support roof
                        //   | support wall
                        //   # dense support
                        //   + less dense support
                        //
                        //     comparing infill            comparing with outline (this is our approach)
                        //    ^^^^^^        ^^^^^^            ^^^^^^            ^^^^^^
                        //    ####||^^      ####||^^          ####||^^          ####||^^
                        //    ######||^^    #####||^^         ######||^^        #####||^^
                        //    ++++####||    ++++##||^         ++++++##||        ++++++||^
                        //    ++++++####    +++++##||         ++++++++##        +++++++||
                        //
                        if (upper_part_boundary_box.hit(this_part_boundary_box))
                        {
                            relevant_upper_polygons.add(upper_infill_parts[upper_part_idx].outline);
                        }
                    }

                    less_dense_support = less_dense_support.intersection(relevant_upper_polygons);
                }
                if (less_dense_support.size() == 0)
                {
                    break;
                }

                // add new infill_area_per_combine_per_density for the current density
                support_infill_part.infill_area_per_combine_per_density.emplace_back();
                std::vector<Polygons>& support_area_current_density = support_infill_part.infill_area_per_combine_per_density.back();
                const Polygons more_dense_support = infill_area.difference(less_dense_support);
                support_area_current_density.push_back(more_dense_support);
            }

            support_infill_part.infill_area_per_combine_per_density.emplace_back();
            std::vector<Polygons>& support_area_current_density = support_infill_part.infill_area_per_combine_per_density.back();
            support_area_current_density.push_back(infill_area);

            assert(support_infill_part.infill_area_per_combine_per_density.size() != 0 && "support_infill_part.infill_area_per_combine_per_density should now be initialized");
#ifdef DEBUG
            for (unsigned int part_i = 0; part_i < support_infill_part.infill_area_per_combine_per_density.size(); ++part_i)
            {
                assert(support_infill_part.infill_area_per_combine_per_density[part_i].size() != 0);
            }
#endif // DEBUG
        }
    }
}

void AreaSupport::combineSupportInfillLayers(SliceDataStorage& storage)
{
    const unsigned int total_layer_count = storage.print_layer_count;
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    // How many support infill layers to combine to obtain the requested sparse thickness.
    const ExtruderTrain& infill_extruder = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const unsigned int combine_layers_amount = std::max(uint64_t(1), round_divide(infill_extruder.getSettingInMicrons("support_infill_sparse_thickness"), std::max(layer_height, (coord_t) 1)));
    if (combine_layers_amount <= 1)
    {
        return;
    }

    /* We need to round down the layer index we start at to the nearest
    divisible index. Otherwise we get some parts that have infill at divisible
    layers and some at non-divisible layers. Those layers would then miss each
    other. */
    size_t min_layer = combine_layers_amount - 1;
    min_layer -= min_layer % combine_layers_amount;  // Round upwards to the nearest layer divisible by infill_sparse_combine.
    size_t max_layer = total_layer_count < storage.support.supportLayers.size() ? total_layer_count : storage.support.supportLayers.size();
    max_layer = max_layer - 1;
    max_layer -= max_layer % combine_layers_amount;  // Round downwards to the nearest layer divisible by infill_sparse_combine.

    for (size_t layer_idx = min_layer; layer_idx <= max_layer; layer_idx += combine_layers_amount) //Skip every few layers, but extrude more.
    {
        if (layer_idx >= storage.support.supportLayers.size())
        {
            break;
        }

        SupportLayer& layer = storage.support.supportLayers[layer_idx];
        for (unsigned int combine_count_here = 1; combine_count_here < combine_layers_amount; ++combine_count_here)
        {
            if (layer_idx < combine_count_here)
            {
                break;
            }

            size_t lower_layer_idx = layer_idx - combine_count_here;
            if (lower_layer_idx < min_layer)
            {
                break;
            }
            SupportLayer& lower_layer = storage.support.supportLayers[lower_layer_idx];

            for (SupportInfillPart& part : layer.support_infill_parts)
            {
                if (part.getInfillArea().empty())
                {
                    continue;
                }
                for (unsigned int density_idx = 0; density_idx < part.infill_area_per_combine_per_density.size(); ++density_idx)
                { // go over each density of gradual infill (these density areas overlap!)
                    std::vector<Polygons>& infill_area_per_combine = part.infill_area_per_combine_per_density[density_idx];
                    Polygons result;
                    for (SupportInfillPart& lower_layer_part : lower_layer.support_infill_parts)
                    {
                        if (! part.outline_boundary_box.hit(lower_layer_part.outline_boundary_box))
                        {
                            continue;
                        }

                        Polygons intersection = infill_area_per_combine[combine_count_here - 1].intersection(lower_layer_part.getInfillArea()).offset(-200).offset(200);
                        if (intersection.size() <= 0)
                        {
                            continue;
                        }

                        result.add(intersection); // add area to be thickened
                        infill_area_per_combine[combine_count_here - 1] = infill_area_per_combine[combine_count_here - 1].difference(intersection); // remove thickened area from less thick layer here

                        unsigned int max_lower_density_idx = density_idx;
                        // Generally: remove only from *same density* areas on layer below
                        // If there are no same density areas, then it's ok to print them anyway
                        // Don't remove other density areas
                        if (density_idx == part.infill_area_per_combine_per_density.size() - 1)
                        {
                            // For the most dense areas on a given layer the density of that area is doubled.
                            // This means that - if the lower layer has more densities -
                            // all those lower density lines are included in the most dense of this layer.
                            // We therefore compare the most dense are on this layer with all densities
                            // of the lower layer with the same or higher density index
                            max_lower_density_idx = lower_layer_part.infill_area_per_combine_per_density.size() - 1;
                        }
                        for (unsigned int lower_density_idx = density_idx; lower_density_idx <= max_lower_density_idx && lower_density_idx < lower_layer_part.infill_area_per_combine_per_density.size(); lower_density_idx++)
                        {
                            std::vector<Polygons>& lower_infill_area_per_combine = lower_layer_part.infill_area_per_combine_per_density[lower_density_idx];
                            lower_infill_area_per_combine[0] = lower_infill_area_per_combine[0].difference(intersection); // remove thickened area from lower (single thickness) layer
                        }
                    }
                    infill_area_per_combine.push_back(result);
                }
            }
        }
    }
}

void AreaSupport::cleanup(SliceDataStorage& storage)
{
    const coord_t support_line_width = storage.getSettingInMicrons("support_line_width");
    for (unsigned int layer_nr = 0; layer_nr < storage.support.supportLayers.size(); layer_nr++)
    {
        SupportLayer& layer = storage.support.supportLayers[layer_nr];
        for (unsigned int part_idx = 0; part_idx < layer.support_infill_parts.size(); part_idx++)
        {
            SupportInfillPart& part = layer.support_infill_parts[part_idx];
            bool can_be_removed = true;
            if (part.inset_count_to_generate > 0)
            {
                can_be_removed = false;
            }
            else
            {
                for (const std::vector<Polygons>& infill_area_per_combine_this_density : part.infill_area_per_combine_per_density)
                {
                    for (const Polygons& infill_area_this_combine_this_density : infill_area_per_combine_this_density)
                    {
                        // remove small areas which were intorduced by rounding errors in comparing the same area on two consecutive layer
                        if (!infill_area_this_combine_this_density.empty() && infill_area_this_combine_this_density.area() > support_line_width * support_line_width)
                        {
                            can_be_removed = false;
                            break;
                        }
                    }
                    if (!can_be_removed)
                    { // break outer loop
                        break;
                    }
                }
            }
            if (can_be_removed)
            {
                part = std::move(layer.support_infill_parts.back());
                layer.support_infill_parts.pop_back();
                part_idx--;
            }
        }
    }
}

Polygons AreaSupport::join(const SliceDataStorage& storage, const Polygons& supportLayer_up, Polygons& supportLayer_this, int64_t supportJoinDistance, int64_t smoothing_distance, int max_smoothing_angle, bool conical_support, int64_t conical_support_offset, int64_t conical_smallest_breadth)
{
    Polygons joined;
    if (conical_support)
    {
        Polygons insetted = supportLayer_up.offset(-conical_smallest_breadth / 2);
        Polygons small_parts = supportLayer_up.difference(insetted.offset(conical_smallest_breadth / 2 + 20));
        joined = supportLayer_this.unionPolygons(supportLayer_up.offset(conical_support_offset))
            .unionPolygons(small_parts);
    }
    else
    {
        joined = supportLayer_this.unionPolygons(supportLayer_up);
    }
    // join different parts
    if (supportJoinDistance > 0)
    {
        joined = joined.offset(supportJoinDistance)
            .offset(-supportJoinDistance);
    }

    // remove jagged line pieces introduced by unioning separate overhang areas for consectuive layers
    //
    // support may otherwise look like:
    //      _____________________      .
    //     /                     \      } dist_from_lower_layer
    //    /__                   __\    /
    //      /''--...........--''\        `\                                                 .
    //     /                     \         } dist_from_lower_layer
    //    /__                   __\      ./
    //      /''--...........--''\     `\                                                    .
    //     /                     \      } dist_from_lower_layer
    //    /_______________________\   ,/
    //            rather than
    //      _____________________
    //     /                     \                                                          .
    //    /                       \                                                         .
    //    |                       |
    //    |                       |
    //    |                       |
    //    |                       |
    //    |                       |
    //    |_______________________|
    //
    // dist_from_lower_layer may be up to max_dist_from_lower_layer (see below), but that value may be extremely high
    joined = joined.smooth_outward(max_smoothing_angle, smoothing_distance);

    return joined;
}

void AreaSupport::generateOverhangAreas(SliceDataStorage& storage)
{
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }
        // it actually also initializes some buffers that are needed in generateSupport
        generateOverhangAreasForMesh(storage, mesh);
    }
}

void AreaSupport::generateSupportAreas(SliceDataStorage& storage)
{
    std::vector<Polygons> global_support_areas_per_layer;
    global_support_areas_per_layer.resize(storage.print_layer_count);

    int max_layer_nr_support_mesh_filled;
    for (max_layer_nr_support_mesh_filled = storage.support.supportLayers.size() - 1; max_layer_nr_support_mesh_filled >= 0; max_layer_nr_support_mesh_filled--)
    {
        const SupportLayer& support_layer = storage.support.supportLayers[max_layer_nr_support_mesh_filled];
        if (support_layer.support_mesh.size() > 0 || support_layer.support_mesh_drop_down.size() > 0)
        {
            break;
        }
    }

    storage.support.layer_nr_max_filled_layer = max_layer_nr_support_mesh_filled;
    for (int layer_nr = 0; layer_nr < max_layer_nr_support_mesh_filled; layer_nr++)
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_nr];
        support_layer.anti_overhang = support_layer.anti_overhang.unionPolygons();
        support_layer.support_mesh_drop_down = support_layer.support_mesh_drop_down.unionPolygons();
        support_layer.support_mesh = support_layer.support_mesh.unionPolygons();
    }

    // initialization of supportAreasPerLayer
    if (storage.print_layer_count > storage.support.supportLayers.size())
    { // there might already be anti_overhang_area data or support mesh data in the supportLayers
        storage.support.supportLayers.resize(storage.print_layer_count);
    }

    // generate support areas
    bool support_meshes_drop_down_handled = false;
    bool support_meshes_handled = false;
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }
        SettingsBaseVirtual* infill_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* roof_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* bottom_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* roof_skin_settings = &storage.meshes[mesh_idx];
        SettingsBaseVirtual* bottom_skin_settings = &storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("support_mesh"))
        {
            if ((mesh.getSettingBoolean("support_mesh_drop_down") && support_meshes_drop_down_handled) ||
                (!mesh.getSettingBoolean("support_mesh_drop_down") && support_meshes_handled) )
            { // handle all support_mesh and support_mesh_drop_down areas only once
                continue;
            }
            // use extruder train settings rather than the per-object settings of the first support mesh encountered.
            // because all support meshes are processed at the same time it doesn't make sense to use the per-object settings of the first support mesh encountered.
            // instead we must use the support extruder settings, which is the settings base common to all support meshes.
            int roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
            int bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
            int infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
            int roof_skin_extruder_nr = storage.getSettingAsIndex("support_roof_skin_extruder_nr");
            int bottom_skin_extruder_nr = storage.getSettingAsIndex("support_bottom_skin_extruder_nr");

            infill_settings = storage.meshgroup->getExtruderTrain(infill_extruder_nr);
            roof_settings = storage.meshgroup->getExtruderTrain(roof_extruder_nr);
            bottom_settings = storage.meshgroup->getExtruderTrain(bottom_extruder_nr);
            roof_skin_settings = storage.meshgroup->getExtruderTrain(roof_skin_extruder_nr);
            bottom_skin_settings = storage.meshgroup->getExtruderTrain(bottom_skin_extruder_nr);

            if (mesh.getSettingBoolean("support_mesh_drop_down"))
            {
                support_meshes_drop_down_handled = true;
            }
            else
            {
                support_meshes_handled = true;
            }
        }
        std::vector<Polygons> mesh_support_areas_per_layer;
        mesh_support_areas_per_layer.resize(storage.print_layer_count, Polygons());

        generateSupportAreasForMesh(storage, *infill_settings, *roof_settings, *bottom_settings, *roof_skin_settings, *bottom_skin_settings, mesh_idx, storage.print_layer_count, mesh_support_areas_per_layer);
        for (unsigned int layer_idx = 0; layer_idx < storage.print_layer_count; layer_idx++)
        {
            global_support_areas_per_layer[layer_idx].add(mesh_support_areas_per_layer[layer_idx]);
        }
    }

    for (unsigned int layer_idx = 0; layer_idx < storage.print_layer_count ; layer_idx++)
    {
        Polygons& support_areas = global_support_areas_per_layer[layer_idx];
        support_areas = support_areas.unionPolygons();
    }

    // handle support interface
    for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
    {
        SliceMeshStorage& mesh = storage.meshes[mesh_idx];
        if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
        {
            continue;
        }

        if (mesh.getSettingBoolean("support_roof_enable"))
        {
            generateSupportRoof(storage, mesh, global_support_areas_per_layer);
        }
        if (mesh.getSettingBoolean("support_bottom_enable"))
        {
            generateSupportBottom(storage, mesh, global_support_areas_per_layer);
        }

		if (mesh.getSettingBoolean("support_roof_enable") /* && storage.getSettingAsIndex("support_roof_extruder_nr") != storage.getSettingAsIndex("support_roof_skin_extruder_nr")*/)
		{
			generateSupportRoofInterface(storage, mesh, global_support_areas_per_layer);
		}
		if (mesh.getSettingBoolean("support_bottom_enable") /* && storage.getSettingAsIndex("support_bottom_extruder_nr") != storage.getSettingAsIndex("support_bottom_skin_extruder_nr")*/)
		{
			generateSupportBottomInterface(storage, mesh, global_support_areas_per_layer);
		}
    }
    // split the global support areas into parts for later gradual support infill generation
    AreaSupport::splitGlobalSupportAreasIntoSupportInfillParts(storage, global_support_areas_per_layer, storage.print_layer_count);
    precomputeCrossInfillTree(storage);
}

void AreaSupport::precomputeCrossInfillTree(SliceDataStorage& storage)
{
    const ExtruderTrain& infill_extr = *storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const EFillMethod support_pattern = infill_extr.getSettingAsFillMethod("support_pattern");
    if (support_pattern == EFillMethod::CROSS || support_pattern == EFillMethod::CROSS_3D && infill_extr.getSettingInMicrons("support_line_distance") > 0)
    {
        AABB3D aabb;
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
            if (mesh.getSettingBoolean("infill_mesh") || mesh.getSettingBoolean("anti_overhang_mesh"))
            {
                continue;
            }
            SettingsBaseVirtual* infill_settings = &storage.meshes[mesh_idx];
            if (mesh.getSettingBoolean("support_mesh"))
            {
                // use extruder train settings rather than the per-object settings of the first support mesh encountered.
                // because all support meshes are processed at the same time it doesn't make sense to use the per-object settings of the first support mesh encountered.
                // instead we must use the support extruder settings, which is the settings base common to all support meshes.
                int infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
                infill_settings = storage.meshgroup->getExtruderTrain(infill_extruder_nr);
            }
            const coord_t aabb_expansion = infill_settings->getSettingInMicrons("support_offset");
            AABB3D aabb_here(mesh.bounding_box);
            aabb_here.include(aabb_here.min - Point3(-aabb_expansion, -aabb_expansion, 0));
            aabb_here.include(aabb_here.max + Point3(-aabb_expansion, -aabb_expansion, 0));
            aabb.include(aabb_here);
        }

        std::string cross_subdisivion_spec_image_file = infill_extr.getSettingString("cross_support_density_image");
        std::ifstream cross_fs(cross_subdisivion_spec_image_file.c_str());
        if (cross_subdisivion_spec_image_file != "" && cross_fs.good())
        {
            storage.support.cross_fill_provider = new SierpinskiFillProvider(aabb, infill_extr.getSettingInMicrons("support_line_distance"), infill_extr.getSettingInMicrons("support_line_width"), cross_subdisivion_spec_image_file);
        }
        else
        {
            if (cross_subdisivion_spec_image_file != "")
            {
                logError("Cannot find density image \'%s\'.", cross_subdisivion_spec_image_file.c_str());
            }

            storage.support.cross_fill_provider = new SierpinskiFillProvider(aabb, infill_extr.getSettingInMicrons("support_line_distance"), infill_extr.getSettingInMicrons("support_line_width"));
        }
    }
}

void AreaSupport::generateOverhangAreasForMesh(SliceDataStorage& storage, SliceMeshStorage& mesh)
{
    if (!mesh.getSettingBoolean("support_enable") && !mesh.getSettingBoolean("support_mesh"))
    {
        return;
    }

    //Fill the overhang areas with emptiness first, even if it's a support mesh, so that we can request the areas.
    mesh.full_overhang_areas.resize(storage.print_layer_count);
    for (size_t layer_idx = 0; layer_idx < storage.print_layer_count; layer_idx++)
    {
        mesh.overhang_areas.emplace_back();
    }

    //Don't generate overhang areas for support meshes. They are dropped down automatically if desired.
    const bool is_support_modifier = mesh.getSettingBoolean("support_mesh");
    if (is_support_modifier)
    {
        return;
    }

    //Don't generate overhang areas if the Z distance is higher than the objects we're generating support for.
    const coord_t layer_height = storage.getSettingInMicrons("layer_height");
    const coord_t z_distance_top = mesh.getSettingInMicrons("support_top_distance");
    const size_t z_distance_top_layers = std::max(uint64_t(0), round_up_divide(z_distance_top, layer_height)) + 1; //Support must always be 1 layer below overhang.
	
	//const size_t z_distance_top_layers = z_distance_top/layer_height + 1;
    if (z_distance_top_layers + 1 > storage.print_layer_count)
    {
        return;
    }

    //Generate points and lines of overhang (for corners pointing downwards, since they don't have an area to support but still need supporting).
    const coord_t minimum_diameter = mesh.getSettingInMicrons("support_minimal_diameter");
    const bool use_towers = mesh.getSettingBoolean("support_use_towers") && minimum_diameter > 0;
    if (use_towers)
    {
        AreaSupport::detectOverhangPoints(storage, mesh, minimum_diameter);
    }

    //Generate the actual areas and store them in the mesh.
    const double support_angle = mesh.getSettingInAngleRadians("support_angle");
    const double tan_angle = tan(support_angle) - 0.01;  //The X/Y component of the support angle. 0.01 to make 90 degrees work too.
    const coord_t max_dist_from_lower_layer = tan_angle * layer_height; //Maximum horizontal distance that can be bridged.
#pragma omp parallel for default(none) shared(storage, mesh) schedule(dynamic)
    for (unsigned int layer_idx = 1; layer_idx < storage.print_layer_count; layer_idx++)
    {
        std::pair<Polygons, Polygons> basic_and_full_overhang = computeBasicAndFullOverhang(storage, mesh, layer_idx, max_dist_from_lower_layer);
        mesh.overhang_areas[layer_idx] = basic_and_full_overhang.first; //Store the results.
        mesh.full_overhang_areas[layer_idx] = basic_and_full_overhang.second;
    }
}

/* 
 * Algorithm:
 * From top layer to bottom layer:
 * - find overhang by looking at the difference between two consecutive layers
 * - join with support areas from layer above
 * - subtract current layer
 * - use the result for the next lower support layer (without doing XY-distance and Z bottom distance, so that a single support beam may move around the model a bit => more stability)
 * - perform inset using X/Y-distance and bottom Z distance
 * 
 * for support buildplate only: purge all support not connected to build plate
 */
void AreaSupport::generateSupportAreasForMesh(SliceDataStorage& storage, const SettingsBaseVirtual& infill_settings, const SettingsBaseVirtual& roof_settings, const SettingsBaseVirtual& bottom_settings, const SettingsBaseVirtual& roof_skin_settings, const SettingsBaseVirtual& bottom_skin_settings, const size_t mesh_idx, const size_t layer_count, std::vector<Polygons>& support_areas)
{
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];

    const SupportStructure support_structure = mesh.getSettingAsSupportStructure("support_structure");
    const bool is_support_mesh_place_holder = mesh.getSettingBoolean("support_mesh"); // whether this mesh has empty SliceMeshStorage and this function is now called to only generate support for all support meshes
    
    if ((!mesh.getSettingBoolean("support_enable") || support_structure != SupportStructure::NORMAL) && !is_support_mesh_place_holder)
    {
        return;
    }

    const ESupportType support_type = storage.getSettingAsSupportType("support_type");
    if (support_type == ESupportType::NONE && !is_support_mesh_place_holder)
    {
        return;
    }

    // early out
    const coord_t layer_thickness = storage.getSettingInMicrons("layer_height");
    const coord_t z_distance_top = ((mesh.getSettingBoolean("support_roof_enable"))? roof_settings : infill_settings).getSettingInMicrons("support_top_distance");

	size_t layer_z_distance_top = z_distance_top / layer_thickness + 1;
	if (!mesh.getSettingBoolean("support_roof_enable"))
        layer_z_distance_top = round_up_divide(z_distance_top, layer_thickness) + 1;
	const coord_t top_support_layer_thickness = layer_thickness - z_distance_top % layer_thickness;

    if (layer_z_distance_top + 1 > layer_count)
    {
        return;
    }

    //Compute the areas that are disallowed by the X/Y distance.
    std::vector<Polygons> xy_disallowed_per_layer;
    xy_disallowed_per_layer.resize(layer_count);
    std::vector<Polygons> sloped_areas_per_layer;
    sloped_areas_per_layer.resize(layer_count);
    sloped_areas_per_layer[0] = Polygons();
    // simplified processing for bottom layer - just ensure support clears part by XY distance
    const coord_t xy_distance = infill_settings.getSettingInMicrons("support_xy_distance");
    const coord_t xy_distance_overhang = infill_settings.getSettingInMicrons("support_xy_distance_overhang");
    const bool use_xy_distance_overhang = infill_settings.getSettingAsSupportDistPriority("support_xy_overrides_z") == SupportDistPriority::Z_OVERRIDES_XY; // whether to use a different xy distance at overhangs
    const double angle = ((mesh.getSettingBoolean("support_roof_enable"))? roof_settings : infill_settings).getSettingInAngleRadians("support_angle");
    const double tan_angle = tan(angle) - 0.01;  // the XY-component of the supportAngle
    //const double tan_angle = 0;
    constexpr bool no_support = false;
    constexpr bool no_prime_tower = false;
    const coord_t support_line_width = infill_settings.getSettingInMicrons("support_line_width");
    const double sloped_areas_angle = mesh.getSettingInAngleRadians("support_bottom_stair_step_min_slope");
    const coord_t sloped_area_detection_width = 10 + static_cast<coord_t>(layer_thickness / std::tan(sloped_areas_angle)) / 2;
    const double minimum_support_area = mesh.getSettingInMillimeters("minimum_support_area");
    const coord_t min_even_wall_line_width = infill_settings.getSettingInMicrons("min_even_wall_line_width");
    xy_disallowed_per_layer[0] = storage.getLayerOutlines(0, false).offset(xy_distance);

    // The maximum width of an odd wall = 2 * minimum even wall width.
    auto half_min_feature_width = min_even_wall_line_width + 10;

    // for all other layers (of non support meshes) compute the overhang area and possibly use that when calculating the support disallowed area
#pragma omp parallel for default(none) shared(xy_disallowed_per_layer, storage, mesh) schedule(dynamic)
    for (unsigned int layer_idx = 1; layer_idx < layer_count; layer_idx++)
    {
        const Polygons outlines  = storage.getLayerOutlines(layer_idx, false);
        const Polygons outlines0 = storage.getLayerOutlines(layer_idx - 1, false);

        const Polygons tube_lower_layer = storage.getLayerOutlines(layer_idx - 1, false).tubeShape(sloped_area_detection_width, 10);
        const Polygons tube_this_layer = outlines.tubeShape(10, sloped_area_detection_width);

        const Polygons temp_polygons = storage.getLayerOutlines(layer_idx - 1, false)
            .tubeShape(sloped_area_detection_width, 10)
            .intersection(outlines.tubeShape(10, sloped_area_detection_width));

        sloped_areas_per_layer[layer_idx] =
            storage.getLayerOutlines(layer_idx - 1, false)
            .tubeShape(sloped_area_detection_width, 10)
            .intersection(outlines.tubeShape(10, sloped_area_detection_width))
            // Do an opening operation so we're not stuck with tiny patches.
            // The later offset is extended with the line-width, so all patches are merged together if there's less than a line-width between them.
            .offset(-10)
            .offset(10 + sloped_area_detection_width);

        if (!is_support_mesh_place_holder)
        { // don't compute overhang for support meshes
            if (use_xy_distance_overhang) //Z overrides XY distance.
            {
                // we also want to use the min XY distance when the support is resting on a sloped surface so we calculate the area of the
                // layer below that protrudes beyond the current layer's area and combine it with the current layer's overhang disallowed area
                Polygons xy_overhang_disallowed = mesh.overhang_areas[layer_idx].offset(z_distance_top * tan_angle);
                Polygons xy_non_overhang_disallowed = outlines.difference(mesh.overhang_areas[layer_idx].offset(xy_distance)).offset(xy_distance);
                xy_disallowed_per_layer[layer_idx] = xy_overhang_disallowed.unionPolygons(xy_non_overhang_disallowed.unionPolygons(outlines.offset(xy_distance_overhang)));
            }
        }
        if (is_support_mesh_place_holder || !use_xy_distance_overhang)
        {
            xy_disallowed_per_layer[layer_idx] = outlines.offset(xy_distance);
        }
    }

    std::vector<Polygons> tower_roofs;
    Polygons stair_removal; // polygons to subtract from support because of stair-stepping

    const bool is_support_mesh_nondrop_place_holder = is_support_mesh_place_holder && !mesh.getSettingBoolean("support_mesh_drop_down");
    const bool is_support_mesh_drop_down_place_holder = is_support_mesh_place_holder && mesh.getSettingBoolean("support_mesh_drop_down");

    const coord_t bottom_stair_step_width = std::max(static_cast<coord_t>(0), mesh.getSettingInMicrons("support_bottom_stair_step_width"));
    const coord_t join_distance = infill_settings.getSettingInMicrons("support_join_distance");
    const coord_t extension_offset = infill_settings.getSettingInMicrons("support_offset");

    const coord_t tower_diameter = infill_settings.getSettingInMicrons("support_tower_diameter");
    const coord_t minimum_diameter = infill_settings.getSettingInMicrons("support_minimal_diameter");
    const bool use_towers = infill_settings.getSettingBoolean("support_use_towers") && minimum_diameter > 0;

    const double tower_roof_angle = infill_settings.getSettingInAngleRadians("support_tower_roof_angle");
    const double tan_tower_roof_angle = tan(tower_roof_angle);
    const int tower_roof_expansion_distance = layer_thickness / tan_tower_roof_angle;
    
    coord_t smoothing_distance;
    { // compute best smoothing_distance
        const int infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr");
        const ExtruderTrain& infill_train = *storage.meshgroup->getExtruderTrain(infill_extruder_nr);
        const coord_t infill_line_width = infill_train.getSettingInMicrons("support_line_width");
        smoothing_distance = infill_line_width;
        if (mesh.getSettingBoolean("support_roof_enable"))
        {
            const int roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr");
            const ExtruderTrain& roof_train = *storage.meshgroup->getExtruderTrain(roof_extruder_nr);
            const coord_t roof_line_width = roof_train.getSettingInMicrons("support_roof_line_width");
            smoothing_distance = std::max(smoothing_distance, roof_line_width);
        }

        if (mesh.getSettingBoolean("support_bottom_enable"))
        {
            const int bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr");
            const ExtruderTrain& bottom_train = *storage.meshgroup->getExtruderTrain(bottom_extruder_nr);
            const coord_t bottom_line_width = bottom_train.getSettingInMicrons("support_bottom_line_width");
            smoothing_distance = std::max(smoothing_distance, bottom_line_width);
        }
    }

    const coord_t z_distance_bottom = ((mesh.getSettingBoolean("support_bottom_enable"))? bottom_settings : infill_settings).getSettingInMicrons("support_bottom_distance");
    //const size_t bottom_empty_layer_count = std::max(0U, round_up_divide(z_distance_bottom, layer_thickness)); // number of empty layers between support and model
	size_t bottom_empty_layer_count = (z_distance_bottom > 0) ? (z_distance_bottom / layer_thickness) : 0;

	if(!mesh.getSettingBoolean("support_bottom_enable"))
		bottom_empty_layer_count = std::max(uint64_t(0), round_up_divide(z_distance_bottom, layer_thickness));

	const coord_t bottom_support_layer_thickness = layer_thickness - z_distance_bottom % layer_thickness;

    const coord_t bottom_stair_step_height = std::max(static_cast<coord_t>(0), mesh.getSettingInMicrons("support_bottom_stair_step_height"));
    const size_t bottom_stair_step_layer_count = bottom_stair_step_height / layer_thickness + 1; // the difference in layers between two stair steps. One is normal support (not stair-like)

    if (bottom_stair_step_layer_count > 1)
    {
        for (size_t layer_idx = 1; layer_idx < layer_count; layer_idx++)
        {
            if (layer_idx % bottom_stair_step_layer_count != 1)
            {
                sloped_areas_per_layer[layer_idx] = sloped_areas_per_layer[layer_idx].unionPolygons(sloped_areas_per_layer[layer_idx - 1]);
            }
        }
    }

    const double conical_support_angle = infill_settings.getSettingInAngleRadians("support_conical_angle");
    coord_t conical_support_offset;
    if (conical_support_angle > 0)
    { // outward ==> wider base than overhang
        conical_support_offset = -(tan(conical_support_angle) - 0.01) * layer_thickness;
    }
    else
    { // inward ==> smaller base than overhang
        conical_support_offset = (tan(-conical_support_angle) - 0.01) * layer_thickness;
    }
    const bool conical_support = infill_settings.getSettingBoolean("support_conical_enabled") && conical_support_angle != 0;
    const coord_t conical_smallest_breadth = infill_settings.getSettingInMicrons("support_conical_min_width");
    const int supportMinAreaSqrt = infill_settings.getSettingInMicrons("support_minimal_diameter");
	
    for (unsigned int layer_idx = layer_count - 1 - layer_z_distance_top; layer_idx != (unsigned int)-1; layer_idx--)
    {
        Polygons layer_this = mesh.full_overhang_areas[layer_idx + layer_z_distance_top];

        if (extension_offset && !is_support_mesh_place_holder)
        {
            Polygons model_outline = storage.getLayerOutlines(layer_idx, false);
            const coord_t offset_per_step = support_line_width / 2;

            // perform a small offset we don't enlarge small features of the support
            Polygons horizontal_expansion = layer_this;
            for (coord_t offset_cumulative = 0; offset_cumulative <= extension_offset; offset_cumulative += offset_per_step)
            {
                horizontal_expansion = horizontal_expansion.offset(offset_per_step);
                model_outline = model_outline.difference(horizontal_expansion);
                model_outline = model_outline.offset(offset_per_step);
                horizontal_expansion = horizontal_expansion.difference(model_outline);
            }
            layer_this = layer_this.offset(extension_offset);
        }

        if (use_towers && !is_support_mesh_place_holder)
        {
            // handle straight walls
            AreaSupport::handleWallStruts(layer_this, minimum_diameter, tower_diameter);
            // handle towers
            constexpr int z_layer_distance_tower = 1; // start tower directly below overhang point
            AreaSupport::handleTowers(layer_this, tower_roofs, mesh.overhang_points, layer_idx, tower_roof_expansion_distance, tower_diameter, minimum_diameter, layer_count, z_layer_distance_tower);
        }

        if (layer_idx + 1 < layer_count)
        { // join with support from layer up
            const Polygons empty;
            const Polygons* layer_above = (layer_idx < support_areas.size())? &support_areas[layer_idx + 1] : &empty;
            const Polygons model_mesh_on_layer = (layer_idx > 0) && !is_support_mesh_nondrop_place_holder ? storage.getLayerOutlines(layer_idx, false) : empty;
            if (is_support_mesh_nondrop_place_holder)
            {
                layer_above = &empty;
                layer_this = layer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh);
            }
            constexpr int max_smoothing_angle = 135; // maximum angle of inner corners to be smoothed
            layer_this = AreaSupport::join(storage, *layer_above, layer_this, join_distance, smoothing_distance, max_smoothing_angle, conical_support, conical_support_offset, conical_smallest_breadth).difference(model_mesh_on_layer);
        }

        // make towers for small support
        if (use_towers)
        {
            for (PolygonsPart poly : layer_this.splitIntoParts())
            {
                const auto polygon_part = poly.difference(xy_disallowed_per_layer[layer_idx])
                                                .offset(-half_min_feature_width)
                                                .offset(half_min_feature_width);
                const int64_t part_area = polygon_part.area();
                if (part_area == 0 || part_area > supportMinAreaSqrt * supportMinAreaSqrt)
                {
                    continue;
                }
                
                constexpr size_t tower_top_layer_count = 6; // number of layers after which to conclude that a tiny support area needs a tower
                if (layer_idx < layer_count - tower_top_layer_count && layer_idx >= tower_top_layer_count + bottom_empty_layer_count)
                {
                    Polygons tiny_tower_here;
                    tiny_tower_here.add(polygon_part);
                    tower_roofs.emplace_back(tiny_tower_here);
                }
            }
        }

        if (is_support_mesh_drop_down_place_holder && storage.support.supportLayers[layer_idx].support_mesh_drop_down.size() > 0)
        { // handle support mesh which should be supported by more support
            layer_this = layer_this.unionPolygons(storage.support.supportLayers[layer_idx].support_mesh_drop_down);
        }

        if (layer_this.size() > 0)
        {
            layer_this = layer_this.difference(xy_disallowed_per_layer[layer_idx]);
        }

        // move up from model
        moveUpFromModel(storage, stair_removal, sloped_areas_per_layer[layer_idx], layer_this, layer_idx, bottom_empty_layer_count, bottom_stair_step_layer_count, bottom_stair_step_width);

        support_areas[layer_idx] = layer_this;

        Progress::messageProgress(Progress::Stage::SUPPORT, layer_count * (mesh_idx + 1) - layer_idx, layer_count * storage.meshes.size());
    }

    // do stuff for when support on buildplate only
    if (support_type == ESupportType::PLATFORM_ONLY)
    {
        Polygons touching_buildplate = support_areas[0]; // TODO: not working for conical support!

        const AngleRadians conical_support_angle = infill_settings.getSettingInAngleRadians("support_conical_angle");
        coord_t conical_support_offset;
        if (conical_support_angle > 0)
        { // outward ==> wider base than overhang
            conical_support_offset = -(tan(conical_support_angle) - 0.01) * layer_thickness;
        }
        else
        { // inward ==> smaller base than overhang
            conical_support_offset = (tan(-conical_support_angle) - 0.01) * layer_thickness;
        }

        const bool conical_support = infill_settings.getSettingBoolean("support_conical_enabled") && conical_support_angle != 0;
        
        for (unsigned int layer_idx = 1 ; layer_idx < storage.support.supportLayers.size() ; layer_idx++)
        {
            const Polygons& layer = support_areas[layer_idx];
            
            if (conical_support)
            { // with conical support the next layer is allowed to be larger than the previous
                touching_buildplate = touching_buildplate.offset(std::abs(conical_support_offset) + 10, ClipperLib::jtMiter, 10); 
            }
            
            touching_buildplate = layer.intersection(touching_buildplate); // from bottom to top, support areas can only decrease!
            
            support_areas[layer_idx] = touching_buildplate;
        }
    }

    //Enforce top Z distance.
    if (layer_z_distance_top > 1)
    {
        // this is performed after the main support generation loop above, because it affects the joining of polygons
        // if this would be performed in the main loop then some support would not have been generated under the overhangs and consequently no support is generated for that,
        // meaning almost no support would be generated in some cases which definitely need support.
        const int max_checking_layer_idx = std::min(static_cast<int>(storage.support.supportLayers.size())
                                                  , static_cast<int>(layer_count - (layer_z_distance_top - 1)));
        const size_t max_checking_idx_size_t = std::max(0, max_checking_layer_idx);
#pragma omp parallel for default(none) shared(support_areas, storage) schedule(dynamic)
        for (size_t layer_idx = 0; layer_idx < max_checking_idx_size_t; layer_idx++)
        {
            support_areas[layer_idx] = support_areas[layer_idx].difference(storage.getLayerOutlines(layer_idx + layer_z_distance_top - 1, false));
        }
    }

    // Procedure to remove floating support
    for (size_t layer_idx = 1; layer_idx < layer_count - 1; layer_idx++)
    {
        Polygons& layer_this = support_areas[layer_idx];

        if (!layer_this.empty())
        {
            Polygons& layer_below = support_areas[layer_idx - 1];
            Polygons& layer_above = support_areas[layer_idx + 1];
            Polygons surrounding_layer = layer_above.unionPolygons(layer_below);
            layer_this = layer_this.intersection(surrounding_layer);
        }
    }

    for (unsigned int layer_idx = support_areas.size() - 1; layer_idx != (unsigned int)std::max(-1, storage.support.layer_nr_max_filled_layer); layer_idx--)
    {
        if (support_areas[layer_idx].size() > 0)
        {
            storage.support.layer_nr_max_filled_layer = layer_idx;
            break;
        }
    }

    storage.support.generated = true;
}

void AreaSupport::moveUpFromModel(const SliceDataStorage& storage, Polygons& stair_removal, Polygons& sloped_areas, Polygons& support_areas, const int layer_idx, const int bottom_empty_layer_count, const unsigned int bottom_stair_step_layer_count, const coord_t support_bottom_stair_step_width)
{
    if (layer_idx < bottom_empty_layer_count)
    {
        return;
    }
    if (bottom_empty_layer_count <= 0 && bottom_stair_step_layer_count <= 1)
    {
        return;
    }

    int bottom_layer_nr = layer_idx - bottom_empty_layer_count;
    const Polygons bottom_outline = storage.getLayerOutlines(bottom_layer_nr, false);

    Polygons to_be_removed;
    if (bottom_stair_step_layer_count <= 1)
    {
        to_be_removed = bottom_outline;
    }
    else
    {
        to_be_removed = stair_removal.unionPolygons(bottom_outline);
        if (layer_idx % bottom_stair_step_layer_count == 0)
        { // update stairs for next step
            const Polygons supporting_bottom = storage.getLayerOutlines(bottom_layer_nr - 1, false);
            const Polygons allowed_step_width = supporting_bottom.offset(support_bottom_stair_step_width).intersection(sloped_areas);

            int step_bottom_layer_nr = bottom_layer_nr - bottom_stair_step_layer_count + 1;
            if (step_bottom_layer_nr >= 0)
            {
                const Polygons step_bottom_outline = storage.getLayerOutlines(step_bottom_layer_nr, false);
                stair_removal = step_bottom_outline.intersection(allowed_step_width);
            }
            else
            {
                stair_removal = allowed_step_width;
            }
        }
    }
    support_areas = support_areas.difference(to_be_removed);
}

std::pair<Polygons, Polygons> AreaSupport::computeBasicAndFullOverhang(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const unsigned int layer_idx, const int64_t max_dist_from_lower_layer)
{
    const Polygons outlines = mesh.layers[layer_idx].getOutlines(); //得到该层的轮廓

    constexpr double smooth_height = 0.4; //mm
    const auto layers_below = static_cast<LayerIndex>(std::round(smooth_height / mesh.getSettingInMillimeters("layer_height")));

    Polygons outlines_below = storage.getLayerOutlines(layer_idx - 1, false).offset(max_dist_from_lower_layer);

    for (int layer_idx_offset = 2; layer_idx - layer_idx_offset >= 0 && layer_idx_offset <= layers_below; layer_idx_offset ++)
    {
            auto outlines_below_ =
                storage.getLayerOutlines(layer_idx - layer_idx_offset, false)
                .offset(max_dist_from_lower_layer * layer_idx_offset);
            outlines_below = outlines_below.unionPolygons(outlines_below_);
    }

    Polygons basic_overhang = outlines.difference(outlines_below);

    const SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
    if (!support_layer.anti_overhang.empty())
    {
        Polygons merged_polygons = support_layer.anti_overhang.unionPolygons();
        basic_overhang = basic_overhang.difference(merged_polygons);
    }

    Polygons overhang_extended =
        basic_overhang
        // +0.1mm for easier joining with support from layer above
        .offset(max_dist_from_lower_layer * layers_below + MM2INT(0.1));
    Polygons full_overhang = overhang_extended.intersection(outlines);
    return std::make_pair(basic_overhang, full_overhang);
}

void AreaSupport::detectOverhangPoints(const SliceDataStorage& storage, SliceMeshStorage& mesh, const coord_t minimum_diameter)
{
    ExtruderTrain* infill_extr = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"));
    const coord_t support_line_width = infill_extr->getSettingInMicrons("support_line_width");

    mesh.overhang_points.resize(storage.print_layer_count);

    for (size_t layer_idx = 1; layer_idx < storage.print_layer_count; layer_idx++)
    {
        const SliceLayer& layer = mesh.layers[layer_idx];
        const SliceLayer& layer_below = mesh.layers[layer_idx - 1];
        
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.outline.outerPolygon().area() < minimum_diameter * minimum_diameter)
            {
                auto has_support_below = !layer_below.getOutlines().intersection(part.outline).empty();
                if (has_support_below)
                {
                    continue;
                }

                const Polygons overhang = part.outline.difference(storage.support.supportLayers[layer_idx].anti_overhang);
                if (!overhang.empty())
                {
                    mesh.overhang_points[layer_idx].push_back(overhang);
                }
            }
        }
    }
}

void AreaSupport::handleTowers(
    Polygons& supportLayer_this,
    std::vector<Polygons>& towerRoofs,
    std::vector<std::vector<Polygons>>& overhang_points,
    int layer_idx,
    int towerRoofExpansionDistance,
    int supportTowerDiameter,
    int supportMinAreaSqrt,
    int layer_count,
    int z_layer_distance_tower
)
{
    int layer_overhang_point = layer_idx + z_layer_distance_tower;
    if (layer_overhang_point >= layer_count - 1)
    {
        return;
    }
    std::vector<Polygons>& overhang_points_here = overhang_points[layer_overhang_point]; // may be changed if an overhang point has a (smaller) overhang point directly below
    // handle new tower roof tops
    if (overhang_points_here.size() > 0)
    {
        { // make sure we have the lowest point (make polys empty if they have small parts below)
            if (layer_overhang_point < layer_count && overhang_points[layer_overhang_point - 1].size() > 0)
            {
                std::vector<Polygons>& overhang_points_below = overhang_points[layer_overhang_point - 1];
                for (Polygons& poly_here : overhang_points_here)
                {
                    for (const Polygons& poly_below : overhang_points_below)
                    {
                        poly_here = poly_here.difference(poly_below.offset(supportMinAreaSqrt * 2));
                    }
                }
            }
        }
        for (Polygons& poly : overhang_points_here)
        {
            if (poly.size() > 0)
            {
                towerRoofs.push_back(poly);
            }
        }
    }

    // make tower roofs
    for (unsigned int roof_idx = 0; roof_idx < towerRoofs.size(); roof_idx++)
    {
        Polygons& tower_roof = towerRoofs[roof_idx];
        if (tower_roof.size() > 0)
        {
            supportLayer_this = supportLayer_this.unionPolygons(tower_roof);

            if (tower_roof[0].area() < supportTowerDiameter * supportTowerDiameter)
            {
                tower_roof = tower_roof.offset(towerRoofExpansionDistance);
            }
            else
            {
                tower_roof.clear();
            }
        }
    }
}

void AreaSupport::handleWallStruts(
    Polygons& supportLayer_this,
    int supportMinAreaSqrt,
    int supportTowerDiameter)
{
    for (unsigned int p = 0; p < supportLayer_this.size(); p++)
    {
        PolygonRef poly = supportLayer_this[p];
        if (poly.size() < 6) // might be a single wall
        {
            PolygonRef poly = supportLayer_this[p];
            int best = -1;
            int best_length2 = -1;
            for (unsigned int i = 0; i < poly.size(); i++)
            {
                int length2 = vSize2(poly[i] - poly[(i+1) % poly.size()]);
                if (length2 > best_length2)
                {
                    best = i;
                    best_length2 = length2;
                }
            }
            
            if (best_length2 < supportMinAreaSqrt * supportMinAreaSqrt)
            {
                break; // this is a small area, not a wall!
            }

            // an estimate of the width of the area
            int width = sqrt( poly.area() * poly.area() / best_length2 ); // sqrt (a^2 / l^2) instead of a / sqrt(l^2)
            
            // add square tower (strut) in the middle of the wall
            if (width < supportMinAreaSqrt)
            {
                Point mid = (poly[best] + poly[(best+1) % poly.size()] ) / 2;
                Polygons struts;
                PolygonRef strut = struts.newPoly();
                strut.add(mid + Point(supportTowerDiameter /2, supportTowerDiameter /2));
                strut.add(mid + Point(-supportTowerDiameter /2, supportTowerDiameter /2));
                strut.add(mid + Point(-supportTowerDiameter /2, -supportTowerDiameter /2));
                strut.add(mid + Point(supportTowerDiameter /2, -supportTowerDiameter /2));
                supportLayer_this = supportLayer_this.unionPolygons(struts);
            }
        }
    }
}

void AreaSupport::generateSupportBottom(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
    const unsigned int bottom_layer_count = round_divide(mesh.getSettingInMicrons("support_bottom_height"), storage.getSettingInMicrons("layer_height")); //Number of layers in support bottom.
    if (bottom_layer_count <= 0)
    {
        return;
    }
    const unsigned int z_distance_bottom = round_up_divide(mesh.getSettingInMicrons("support_bottom_distance"), storage.getSettingInMicrons("layer_height")); //Number of layers between support bottom and model.
    const unsigned int skip_layer_count = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support bottoms above model.
    const coord_t bottom_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_line_width");
    const coord_t bottom_outline_offset = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_offset");

    const unsigned int scan_count = std::max(1u, (bottom_layer_count - 1) / skip_layer_count); //How many measurements to take to generate bottom areas.
    const float z_skip = std::max(1.0f, float(bottom_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.
    const double minimum_bottom_area = mesh.getSettingInMillimeters("minimum_bottom_area");

    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = z_distance_bottom; layer_idx < support_layers.size(); layer_idx++)
    {
        const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));
        Polygons mesh_outlines;
        for (float layer_idx_below = bottom_layer_idx_below; std::round(layer_idx_below) < (int)(layer_idx - z_distance_bottom); layer_idx_below += z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_below)].getOutlines());
        }
        Polygons bottoms;
        generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, bottom_line_width, bottom_outline_offset, minimum_bottom_area, bottoms);
        support_layers[layer_idx].support_bottom.add(bottoms);
    }
}

void AreaSupport::generateSupportBottomInterface(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
    unsigned int bottom_layer_count = mesh.getSettingAsCount("support_interface_bottom_skin_layer_count"); //Number of layers in support bottom interface.

	const unsigned int z_distance_bottom = mesh.getSettingInMicrons("support_bottom_distance") / storage.getSettingInMicrons("layer_height");

	storage.support.bottom_interface_layer_thickness = storage.getSettingInMicrons("layer_height") - mesh.getSettingInMicrons("support_bottom_distance") % storage.getSettingInMicrons("layer_height");

	const unsigned int skip_layer_count = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support bottoms above model.

	const coord_t bottom_line_width = (mesh.getSettingBoolean("support_bottom_enable")) ? storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_line_width") : storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"))->getSettingInMicrons("support_line_width");

	unsigned int scan_count = 1; //How many measurements to take to generate bottom areas.
	float z_skip = 1.0f; //How many layers to skip between measurements. Using float for better spread, but this is later rounded.

    const coord_t bottom_outline_offset = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_bottom_extruder_nr"))->getSettingInMicrons("support_bottom_offset");
    const double minimum_bottom_area = mesh.getSettingInMillimeters("minimum_bottom_area");

	std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
	for (unsigned int layer_idx = z_distance_bottom; layer_idx < support_layers.size(); layer_idx++)
	{		
		const unsigned int bottom_layer_idx_below = std::max(0, int(layer_idx) - int(bottom_layer_count) - int(z_distance_bottom));//Maximum layer of the model that generates support roof
		Polygons mesh_outlines;
		mesh_outlines.add(mesh.layers[bottom_layer_idx_below].getOutlines());
		Polygons interface_bottoms;

		if (!mesh.getSettingBoolean("support_bottom_enable") || mesh.getSettingInMicrons("support_bottom_height") <= 0)
		{
			generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, bottom_line_width, bottom_outline_offset, minimum_bottom_area, interface_bottoms);
		}
		else
		{
			generateSupportInterfaceLayer(support_layers[layer_idx].support_bottom, mesh_outlines, bottom_line_width, bottom_outline_offset, minimum_bottom_area, interface_bottoms);
		}
		support_layers[layer_idx].support_bottom_interface.add(interface_bottoms);
	}
}

void AreaSupport::generateSupportRoof(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
    const unsigned int roof_layer_count = round_divide(mesh.getSettingInMicrons("support_roof_height"), storage.getSettingInMicrons("layer_height")); //Number of layers in support roof.
    if (roof_layer_count <= 0)
    {
        return;
	}
	const unsigned int z_distance_top = mesh.getSettingInMicrons("support_top_distance")/storage.getSettingInMicrons("layer_height");	
	
	const unsigned int skip_layer_count = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("support_interface_skip_height"), storage.getSettingInMicrons("layer_height"))); //Resolution of generating support roof below model.
    const coord_t roof_line_width = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_line_width");
    const coord_t roof_outline_offset = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_offset");

    const unsigned int scan_count = std::max(1u, (roof_layer_count - 1) / skip_layer_count); //How many measurements to take to generate roof areas.
    const float z_skip = std::max(1.0f, float(roof_layer_count - 1) / float(scan_count)); //How many layers to skip between measurements. Using float for better spread, but this is later rounded.
    const double minimum_roof_area = mesh.getSettingInMillimeters("minimum_roof_area");
    
    std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
    for (unsigned int layer_idx = 0; static_cast<int>(layer_idx) < static_cast<int>(support_layers.size() - z_distance_top); layer_idx++)
    {
        const unsigned int top_layer_idx_above = std::min(static_cast<unsigned int>(support_layers.size() - 1), layer_idx + roof_layer_count + z_distance_top); //Maximum layer of the model that generates support roof.
        Polygons mesh_outlines;
        for (float layer_idx_above = top_layer_idx_above; layer_idx_above > layer_idx + z_distance_top; layer_idx_above -= z_skip)
        {
            mesh_outlines.add(mesh.layers[std::round(layer_idx_above)].getOutlines());
        }
        Polygons roofs;
        generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, roof_line_width, roof_outline_offset, minimum_roof_area, roofs);
        support_layers[layer_idx].support_roof.add(roofs);
    }
}

void AreaSupport::generateSupportRoofInterface(SliceDataStorage& storage, const SliceMeshStorage& mesh, std::vector<Polygons>& global_support_areas_per_layer)
{
	const unsigned int z_distance_top = mesh.getSettingInMicrons("support_top_distance") / storage.getSettingInMicrons("layer_height"); //Number of layers between support roof and model.	
	storage.support.roof_interface_layer_thickness = storage.getSettingInMicrons("layer_height") - mesh.getSettingInMicrons("support_top_distance") % storage.getSettingInMicrons("layer_height");	
	const coord_t roof_line_width = (mesh.getSettingBoolean("support_roof_enable")) ? storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_line_width") : storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_infill_extruder_nr"))->getSettingInMicrons("support_line_width");
    const coord_t roof_outline_offset = storage.meshgroup->getExtruderTrain(storage.getSettingAsIndex("support_roof_extruder_nr"))->getSettingInMicrons("support_roof_offset");

	unsigned int roof_layer_count = mesh.getSettingAsCount("support_interface_top_skin_layer_count");
	const unsigned int scan_count = 1;
	const float z_skip = 1.0f;

    const double minimum_roof_area = mesh.getSettingInMillimeters("minimum_roof_area");
	
	std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
	for (unsigned int layer_idx = 0; static_cast<int>(layer_idx) < static_cast<int>(support_layers.size() - z_distance_top); layer_idx++)
	{
		const unsigned int top_layer_idx_above = std::min(static_cast<unsigned int>(support_layers.size() - 1), layer_idx + roof_layer_count + z_distance_top);
		Polygons mesh_outlines;
		mesh_outlines.add(mesh.layers[top_layer_idx_above].getOutlines());
		Polygons interface_roofs;
		if (!mesh.getSettingBoolean("support_roof_enable") || mesh.getSettingInMicrons("support_roof_height") <= 0)
		{
			generateSupportInterfaceLayer(global_support_areas_per_layer[layer_idx], mesh_outlines, roof_line_width, roof_outline_offset, minimum_roof_area, interface_roofs);
		}
		else
		{
			generateSupportInterfaceLayer(support_layers[layer_idx].support_roof, mesh_outlines, roof_line_width, roof_outline_offset, minimum_roof_area, interface_roofs);
		}
		support_layers[layer_idx].support_roof_interface.add(interface_roofs);
	}
}

void AreaSupport::generateSupportInterfaceLayer(Polygons& support_areas, const Polygons colliding_mesh_outlines, const coord_t safety_offset, const coord_t outline_offset, const double minimum_interface_area, Polygons& interface_polygons)
{
    Polygons model = colliding_mesh_outlines.unionPolygons();
    interface_polygons = support_areas.offset(safety_offset / 2).intersection(model);
    interface_polygons = interface_polygons.offset(safety_offset).intersection(support_areas); //Make sure we don't generate any models that are not printable.
    if (outline_offset != 0)
    {
        interface_polygons = interface_polygons.offset(outline_offset);
        if (outline_offset > 0) // The interface might exceed the area of the normal support.
        {
            interface_polygons = interface_polygons.intersection(support_areas);
        }
    }
    if (minimum_interface_area > 0.0)
    {
        interface_polygons.removeSmallAreas(minimum_interface_area);
    }
    support_areas = support_areas.difference(interface_polygons);
}

}//namespace cura
