//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "FffPolygonGenerator.h"

#include <algorithm>
#include <map> // multimap (ordered map allowing duplicate keys)
#include <fstream> // ifstream.good()

#ifdef _OPENMP
    #include <omp.h>
#endif // _OPENMP

#include "utils/math.h"
#include "utils/algorithm.h"
#include "slicer.h"
#include "utils/gettime.h"
#include "utils/logoutput.h"
#include "MeshGroup.h"
#include "support.h"
#include "multiVolumes.h"
#include "layerPart.h"
#include "Mold.h"
#include "WallsComputation.h"
#include "SkirtBrim.h"
#include "skin.h"
#include "infill/SpaghettiInfill.h"
#include "infill/SpaceFillingTreeFill.h"
#include "infill/SierpinskiFillProvider.h"
#include "infill.h"
#include "raft.h"
#include "progress/Progress.h"
#include "PrintFeature.h"
#include "ConicalOverhang.h"
#include "TopSurface.h"
#include "TreeSupport.h"
#include "progress/ProgressEstimator.h"
#include "progress/ProgressStageEstimator.h"
#include "progress/ProgressEstimatorLinear.h"
#include "settings/AdaptiveLayerHeights.h"


namespace cura
{

    bool FffPolygonGenerator::generateAreas(SliceDataStorage& storage, MeshGroup* meshgroup, TimeKeeper& timeKeeper)
    {
        if (!sliceModel(meshgroup, timeKeeper, storage))
        {
            return false;
        }

        slices2polygons(storage, timeKeeper);

        return true;
    }

    unsigned int FffPolygonGenerator::getDraftShieldLayerCount(const unsigned int total_layers) const
    {
        if (!getSettingBoolean("draft_shield_enabled"))
        {
            return 0;
        }
        switch (getSettingAsDraftShieldHeightLimitation("draft_shield_height_limitation"))
        {
        default:
        case DraftShieldHeightLimitation::FULL:
            return total_layers;
        case DraftShieldHeightLimitation::LIMITED:
            return std::max((coord_t)0, (getSettingInMicrons("draft_shield_height") - getSettingInMicrons("layer_height_0")) / getSettingInMicrons("layer_height") + 1);
        }
    }

    bool FffPolygonGenerator::sliceModel(MeshGroup* meshgroup, TimeKeeper& timeKeeper, SliceDataStorage& storage) /// slices the model
    {
        Progress::messageProgressStage(Progress::Stage::SLICING, &timeKeeper);

        storage.model_min = meshgroup->min();
        storage.model_max = meshgroup->max();
        storage.model_size = storage.model_max - storage.model_min;

        log("Slicing model...\n");

        // regular layers
        int slice_layer_count = 0;
        int layer_thickness = getSettingInMicrons("layer_height");
        int initial_layer_thickness = getSettingInMicrons("layer_height_0");

        // Initial layer height of 0 is not allowed. Negative layer height is nonsense.
        if (initial_layer_thickness <= 0)
        {
            logError("Initial layer height %i is disallowed.\n", initial_layer_thickness);
            return false;
        }

        // Layer height of 0 is not allowed. Negative layer height is nonsense.
        if (layer_thickness <= 0)
        {
            logError("Layer height %i is disallowed.\n", layer_thickness);
            return false;
        }

        // variable layers
        AdaptiveLayerHeights* adaptive_layer_heights = nullptr;
        bool use_variable_layer_heights = getSettingBoolean("adaptive_layer_height_enabled");

        if (use_variable_layer_heights)
        {
            // Calculate adaptive layer heights
            coord_t variable_layer_height_max_variation = getSettingInMicrons("adaptive_layer_height_variation");
            coord_t variable_layer_height_variation_step = getSettingInMicrons("adaptive_layer_height_variation_step");
            double adaptive_threshold = getSettingInAngleDegrees("adaptive_layer_height_threshold");
            adaptive_layer_heights = new AdaptiveLayerHeights(meshgroup, layer_thickness, initial_layer_thickness,
                variable_layer_height_max_variation,
                variable_layer_height_variation_step, adaptive_threshold);

            // Get the amount of layers
            slice_layer_count = adaptive_layer_heights->getLayerCount();
        }
        else
        {
            slice_layer_count = (storage.model_max.z - initial_layer_thickness) / layer_thickness + 2;
        }

        // Model is shallower than layer_height_0, so not even the first layer is sliced. Return an empty model then.
        if (slice_layer_count <= 0)
        {
            return true; // This is NOT an error state!
        }

        std::vector<Slicer*> slicerList;
        for (unsigned int mesh_idx = 0; mesh_idx < meshgroup->meshes.size(); mesh_idx++)
        {
            // Check if adaptive layers is populated to prevent accessing a method on NULL
            std::vector<AdaptiveLayer>* adaptive_layer_height_values = {};
            if (adaptive_layer_heights != NULL)
            {
                adaptive_layer_height_values = adaptive_layer_heights->getLayers();
            }

            Mesh& mesh = meshgroup->meshes[mesh_idx];
            Slicer* slicer = new Slicer(&mesh, initial_layer_thickness, layer_thickness, slice_layer_count,
                mesh.getSettingBoolean("meshfix_keep_open_polygons"),
                mesh.getSettingBoolean("meshfix_extensive_stitching"),
                use_variable_layer_heights, adaptive_layer_height_values);

            slicerList.push_back(slicer);

            Progress::messageProgress(Progress::Stage::SLICING, mesh_idx + 1, meshgroup->meshes.size());
        }

        // Clear the mesh face and vertex data, it is no longer needed after this point, and it saves a lot of memory.
        meshgroup->clear();

        Mold::process(storage, slicerList, layer_thickness);

        for (unsigned int mesh_idx = 0; mesh_idx < slicerList.size(); mesh_idx++)
        {
            Mesh& mesh = storage.meshgroup->meshes[mesh_idx];
            if (mesh.getSettingBoolean("conical_overhang_enabled") && !mesh.getSettingBoolean("anti_overhang_mesh"))
            {
                ConicalOverhang::apply(slicerList[mesh_idx], mesh.getSettingInAngleRadians("conical_overhang_angle"), layer_thickness);
            }
        }

        MultiVolumes::carveCuttingMeshes(slicerList, storage.meshgroup->meshes);

        Progress::messageProgressStage(Progress::Stage::PARTS, &timeKeeper);

        if (storage.getSettingBoolean("carve_multiple_volumes"))
        {
            carveMultipleVolumes(slicerList, storage.getSettingBoolean("alternate_carve_order"));
        }

        generateMultipleVolumesOverlap(slicerList);

        storage.print_layer_count = 0;
        for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
        {
            Mesh& mesh = storage.meshgroup->meshes[meshIdx];
            Slicer* slicer = slicerList[meshIdx];
            if (!mesh.getSettingBoolean("anti_overhang_mesh") && !mesh.getSettingBoolean("infill_mesh") && !mesh.getSettingBoolean("cutting_mesh"))
            {
                storage.print_layer_count = std::max(storage.print_layer_count, slicer->layers.size());
            }
        }
        storage.support.supportLayers.resize(storage.print_layer_count);

        storage.meshes.reserve(slicerList.size()); // causes there to be no resize in meshes so that the pointers in sliceMeshStorage._config to retraction_config don't get invalidated.
        for (unsigned int meshIdx = 0; meshIdx < slicerList.size(); meshIdx++)
        {
            Slicer* slicer = slicerList[meshIdx];
            Mesh& mesh = storage.meshgroup->meshes[meshIdx];

            // always make a new SliceMeshStorage, so that they have the same ordering / indexing as meshgroup.meshes
            storage.meshes.emplace_back(&meshgroup->meshes[meshIdx], meshIdx, slicer->layers.size()); // new mesh in storage had settings from the Mesh 对storage.meshes开辟空间-----layers的空间，只是开辟，没有赋值操作。
            SliceMeshStorage& meshStorage = storage.meshes.back();
            // only create layer parts for normal meshes
            const bool is_support_modifier = AreaSupport::handleSupportModifierMesh(storage, mesh, slicer);
            if (!is_support_modifier)
            {
                createLayerParts(meshStorage, slicer, mesh.getSettingBoolean("meshfix_union_all"), mesh.getSettingBoolean("meshfix_union_all_remove_holes"));
            }

            // check one if raft offset is needed
            bool has_raft = getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT;

            // calculate the height at which each layer is actually printed (printZ)
            for (unsigned int layer_nr = 0; layer_nr < meshStorage.layers.size(); layer_nr++)
            {
                SliceLayer& layer = meshStorage.layers[layer_nr];

                if (use_variable_layer_heights)
                {
                    meshStorage.layers[layer_nr].printZ = adaptive_layer_heights->getLayers()->at(layer_nr).z_position;
                    meshStorage.layers[layer_nr].thickness = adaptive_layer_heights->getLayers()->at(layer_nr).layer_height;
                }
                else
                {
                    meshStorage.layers[layer_nr].printZ = initial_layer_thickness + (layer_nr * layer_thickness);
                    if (layer_nr == 0){
                        meshStorage.layers[layer_nr].thickness = initial_layer_thickness;
                    }
                    else{
                        meshStorage.layers[layer_nr].thickness = layer_thickness;
                    }
                }

                // add the raft offset to each layer
                if (has_raft)
                {
                    ExtruderTrain* train = storage.meshgroup->getExtruderTrain(getSettingAsIndex("raft_skin_extruder_nr"));
                    layer.printZ +=
                        Raft::getTotalThickness(storage)
                        + train->getSettingInMicrons("raft_airgap")
                        - train->getSettingInMicrons("layer_0_z_overlap");

                    if (layer_nr == 0){
                        layer.printZ += train->getSettingInMicrons("layer_0_z_overlap");
                    }
                }
            }

            delete slicerList[meshIdx];

            Progress::messageProgress(Progress::Stage::PARTS, meshIdx + 1, slicerList.size());
        }
        return true;
    }

    void FffPolygonGenerator::slices2polygons(SliceDataStorage& storage, TimeKeeper& time_keeper)
    {
        // compute layer count and remove first empty layers
        // there is no separate progress stage for removeEmptyFisrtLayer (TODO)
        unsigned int slice_layer_count = 0;
        for (SliceMeshStorage& mesh : storage.meshes){
            if (!mesh.getSettingBoolean("infill_mesh") && !mesh.getSettingBoolean("anti_overhang_mesh")){
                slice_layer_count = std::max<unsigned int>(slice_layer_count, mesh.layers.size());
            }
        }

        // handle meshes
        std::vector<double> mesh_timings;
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++){
            mesh_timings.push_back(1.0); // TODO: have a more accurate estimate of the relative time it takes per mesh, based on the height and number of polygons
        }
        ProgressStageEstimator inset_skin_progress_estimate(mesh_timings);

        Progress::messageProgressStage(Progress::Stage::INSET_SKIN, &time_keeper);
        std::vector<unsigned int> mesh_order;
        { // compute mesh order
            std::multimap<int, unsigned int> order_to_mesh_indices;
            for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
            {
                order_to_mesh_indices.emplace(storage.meshes[mesh_idx].getSettingAsIndex("infill_mesh_order"), mesh_idx);
            }
            for (std::pair<const int, unsigned int>& order_and_mesh_idx : order_to_mesh_indices)
            {
                mesh_order.push_back(order_and_mesh_idx.second);
            }
        }
        for (unsigned int mesh_order_idx(0); mesh_order_idx < mesh_order.size(); ++mesh_order_idx)
        {
            processBasicWallsSkinInfill(storage, mesh_order_idx, mesh_order, inset_skin_progress_estimate);
            Progress::messageProgress(Progress::Stage::INSET_SKIN, mesh_order_idx + 1, storage.meshes.size());
        }

        if (isEmptyLayer(storage, 0) && !isEmptyLayer(storage, 1)){
            // the first layer is empty, the second is not empty, so remove the empty first layer as support isn't going to be generated under it.
            // Do this irrespective of the value of remove_empty_first_layers as that setting is hidden when support is enabled and so cannot be relied upon
            removeEmptyFirstLayers(storage, getSettingInMicrons("layer_height"), storage.print_layer_count); // changes storage.print_layer_count!
        }
        log("Layer count: %i\n", storage.print_layer_count);
        Progress::messageProgressStage(Progress::Stage::SUPPORT, &time_keeper);
        AreaSupport::generateOverhangAreas(storage);
        AreaSupport::generateSupportAreas(storage);
        TreeSupport tree_support_generator(storage);
        tree_support_generator.generateSupportAreas(storage);

        // we need to remove empty layers after we have processed the insets
        // processWalls might throw away parts if they have no wall at all (cause it doesn't fit)
        // brim depends on the first layer not being empty
        // only remove empty layers if we haven't generate support, because then support was added underneath the model.
        // for some materials it's better to print on support than on the build plate.
        if (getSettingBoolean("remove_empty_first_layers")){
            removeEmptyFirstLayers(storage, getSettingInMicrons("layer_height"), storage.print_layer_count); // changes storage.print_layer_count!
        }
        if (storage.print_layer_count == 0){
            log("Stopping process because there are no non-empty layers.\n");
            return;
        }

        computePrintHeightStatistics(storage);

        // handle helpers
        if (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED){
            storage.primeTower.generateGroundpoly(storage);
            storage.primeTower.generatePaths(storage);
            storage.primeTower.subtractFromSupport(storage);
        }

        if (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::INTERLACED){
            storage.interlacedprimeTower.generatePaths(storage);
            storage.interlacedprimeTower.subtractFromSupport(storage);
        }

        logDebug("Processing ooze shield\n");
        processOozeShield(storage);

        logDebug("Processing draft shield\n");
        processDraftShield(storage);

        // This catches a special case in which the models are in the air, and then
        // the adhesion mustn't be calculated.
        if (!isEmptyLayer(storage, 0) || (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::NESTED && storage.primeTower.enabled) ||
            (getSettingAsPrimeTowerType("prime_tower_type") == PrimeTowerType::INTERLACED && storage.interlacedprimeTower.enabled))
        {
            log("Processing platform adhesion\n");
            processPlatformAdhesion(storage);
        }

        logDebug("Processing gaps\n");
        logDebug("Meshes post-processing\n");
        // meshes post processing
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            processDerivedWallsSkinInfill(mesh);
        }

        logDebug("Processing gradual support\n");
        // generate gradual support
        AreaSupport::generateSupportInfillFeatures(storage);
    }

void FffPolygonGenerator::processBasicWallsSkinInfill(SliceDataStorage& storage, unsigned int mesh_order_idx, std::vector<unsigned int>& mesh_order, ProgressStageEstimator& inset_skin_progress_estimate)
{
    unsigned int mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    size_t mesh_layer_count = mesh.layers.size();
    if (mesh.getSettingBoolean("infill_mesh"))
    {
        processInfillMesh(storage, mesh_order_idx, mesh_order);
    }

    // TODO: make progress more accurate!!
    // note: estimated time for insets : skins = 22.953 : 48.858
    std::vector<double> walls_vs_skin_timing({ 22.953, 48.858 });
    ProgressStageEstimator* mesh_inset_skin_progress_estimator = new ProgressStageEstimator(walls_vs_skin_timing);

    inset_skin_progress_estimate.nextStage(mesh_inset_skin_progress_estimator); // the stage of this function call

    ProgressEstimatorLinear* inset_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(inset_estimator);

    //walls
    unsigned int processed_layer_count = 0;
#pragma omp parallel for default(none) shared(mesh_layer_count, storage, mesh, inset_skin_progress_estimate, processed_layer_count) schedule(dynamic)
    for (unsigned int layer_number = 0; layer_number < mesh.layers.size(); layer_number++)
    {
        logDebug("Processing insets for layer %i of %i\n", layer_number, mesh_layer_count);
        processWalls(storage, mesh, layer_number);
#ifdef _OPENMP
        if (omp_get_thread_num() == 0)
#endif
        { // progress estimation is done only in one thread so that no two threads message progress at the same time
            int _processed_layer_count;
#pragma omp atomic read
            _processed_layer_count = processed_layer_count;
            double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
            Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
        }
#pragma omp atomic
        processed_layer_count++;
    }
    ProgressEstimatorLinear* skin_estimator = new ProgressEstimatorLinear(mesh_layer_count);
    mesh_inset_skin_progress_estimator->nextStage(skin_estimator);

    bool process_infill = mesh.getSettingInMicrons("infill_line_distance") > 0;
    if (!process_infill)
    { // do process infill anyway if it's modified by modifier meshes
        for (unsigned int other_mesh_order_idx(mesh_order_idx + 1); other_mesh_order_idx < mesh_order.size(); ++other_mesh_order_idx)
        {
            unsigned int other_mesh_idx = mesh_order[other_mesh_order_idx];
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (other_mesh.getSettingBoolean("infill_mesh"))
            {
                AABB3D aabb = storage.meshgroup->meshes[mesh_idx].getAABB();
                AABB3D other_aabb = storage.meshgroup->meshes[other_mesh_idx].getAABB();
                if (aabb.hit(other_aabb))
                {
                    process_infill = true;
                }
            }
        }
    }
    // skin & infill
    // Progress::messageProgressStage(Progress::Stage::SKIN, &time_keeper);
    int mesh_max_bottom_layer_count = 0;
    if (getSettingBoolean("magic_spiralize"))
    {
        mesh_max_bottom_layer_count = std::max(mesh_max_bottom_layer_count, mesh.getSettingAsCount("bottom_layers"));
    }

    processed_layer_count = 0;
#pragma omp parallel default(none) shared(mesh_layer_count, storage, mesh, mesh_max_bottom_layer_count, process_infill, inset_skin_progress_estimate, processed_layer_count)
    {
#pragma omp for schedule(dynamic)
        for (unsigned int layer_number = 0; layer_number < mesh.layers.size(); layer_number++)
        {
            logDebug("Processing skins and infill layer %i of %i\n", layer_number, mesh_layer_count);
            if (!getSettingBoolean("magic_spiralize") || static_cast<int>(layer_number) < mesh_max_bottom_layer_count)    //Only generate up/downskin and infill for the first X layers when spiralize is choosen.
            {
                processSkinsAndInfill(storage, mesh, layer_number, process_infill);
            }
#ifdef _OPENMP
            if (omp_get_thread_num() == 0)
#endif
            { // progress estimation is done only in one thread so that no two threads message progress at the same time
                int _processed_layer_count;
#pragma omp atomic read
                _processed_layer_count = processed_layer_count;
                double progress = inset_skin_progress_estimate.progress(_processed_layer_count);
                Progress::messageProgress(Progress::Stage::INSET_SKIN, progress * 100, 100);
            }
#pragma omp atomic
            processed_layer_count++;
        }
    }
}

void FffPolygonGenerator::processInfillMesh(SliceDataStorage& storage, unsigned int mesh_order_idx, std::vector<unsigned int>& mesh_order)
{
    unsigned int mesh_idx = mesh_order[mesh_order_idx];
    SliceMeshStorage& mesh = storage.meshes[mesh_idx];
    mesh.layer_nr_max_filled_layer = -1;
    for (unsigned int layer_idx = 0; layer_idx < mesh.layers.size(); layer_idx++)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        std::vector<PolygonsPart> new_parts;

        for (unsigned int other_mesh_idx : mesh_order)
        { // limit the infill mesh's outline to within the infill of all meshes with lower order
            if (other_mesh_idx == mesh_idx)
            {
                break; // all previous meshes have been processed
            }
            SliceMeshStorage& other_mesh = storage.meshes[other_mesh_idx];
            if (layer_idx >= other_mesh.layers.size())
            { // there can be no interaction between the infill mesh and this other non-infill mesh
                continue;
            }

            SliceLayer& other_layer = other_mesh.layers[layer_idx];

            for (SliceLayerPart& part : layer.parts)
            {
                for (SliceLayerPart& other_part : other_layer.parts)
                { // limit the outline of each part of this infill mesh to the infill of parts of the other mesh with lower infill mesh order
                    if (!part.boundaryBox.hit(other_part.boundaryBox))
                    { // early out
                        continue;
                    }
                    Polygons new_outline = part.outline.intersection(other_part.getOwnInfillArea());
                    if (new_outline.size() == 1)
                    { // we don't have to call splitIntoParts, because a single polygon can only be a single part
                        PolygonsPart outline_part_here;
                        outline_part_here.add(new_outline[0]);
                        new_parts.push_back(outline_part_here);
                    }
                    else if (new_outline.size() > 1)
                    { // we don't know whether it's a multitude of parts because of newly introduced holes, or because the polygon has been split up
                        std::vector<PolygonsPart> new_parts_here = new_outline.splitIntoParts();
                        for (PolygonsPart& new_part_here : new_parts_here)
                        {
                            new_parts.push_back(new_part_here);
                        }
                    }
                    // change the infill area of the non-infill mesh which is to be filled with e.g. lines
                    other_part.infill_area_own = other_part.getOwnInfillArea().difference(part.outline);
                    // note: don't change the part.infill_area, because we change the structure of that area, while the basic area in which infill is printed remains the same
                    //       the infill area remains the same for combing
                }
            }
        }

        layer.parts.clear();
        for (PolygonsPart& part : new_parts)
        {
            layer.parts.emplace_back();
            layer.parts.back().outline = part;
            layer.parts.back().boundaryBox.calculate(part);
        }

        if (layer.parts.size() > 0 || (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0) )
        {
            mesh.layer_nr_max_filled_layer = layer_idx; // last set by the highest non-empty layer
        }
    }
}

void FffPolygonGenerator::processDerivedWallsSkinInfill(SliceMeshStorage& mesh)
{
    if (mesh.getSettingBoolean("infill_support_enabled"))
    {
        SkinInfillAreaComputation::generateInfillSupport(mesh);
    }

    // generate spaghetti infill filling areas and volumes
    if (mesh.getSettingBoolean("spaghetti_infill_enabled"))
    {
        SpaghettiInfill::generateSpaghettiInfill(mesh);
    }
    else
    {
        // create gradual infill areas
        SkinInfillAreaComputation::generateGradualInfill(mesh, mesh.getSettingInMicrons("gradual_infill_step_height"), mesh.getSettingAsCount("gradual_infill_steps"));

        //SubDivCube Pre-compute Octree
        if (mesh.getSettingInMicrons("infill_line_distance") > 0
            && mesh.getSettingAsFillMethod("infill_pattern") == EFillMethod::CUBICSUBDIV)
        {
            const Point3 mesh_middle = mesh.bounding_box.getMiddle();
            const Point infill_origin(mesh_middle.x + mesh.getSettingInMicrons("infill_offset_x"), mesh_middle.y + mesh.getSettingInMicrons("infill_offset_y"));
            SubDivCube::precomputeOctree(mesh, infill_origin);
        }

        // Pre-compute Cross Fractal
        if (mesh.getSettingInMicrons("infill_line_distance") > 0
            && (mesh.getSettingAsFillMethod("infill_pattern") == EFillMethod::CROSS
                || mesh.getSettingAsFillMethod("infill_pattern") == EFillMethod::CROSS_3D)
        )
        {
            const std::string cross_subdivision_spec_image_file = mesh.getSettingString("cross_infill_density_image");
            std::ifstream cross_fs(cross_subdivision_spec_image_file.c_str());
            if (!cross_subdivision_spec_image_file.empty() && cross_fs.good())
            {
                mesh.cross_fill_provider = new SierpinskiFillProvider(mesh.bounding_box, mesh.getSettingInMicrons("infill_line_distance"), mesh.getSettingInMicrons("infill_line_width"), cross_subdivision_spec_image_file);
            }
            else
            {
                if (!cross_subdivision_spec_image_file.empty() && cross_subdivision_spec_image_file != " ")
                {
                    logError("Cannot find density image \'%s\'.", cross_subdivision_spec_image_file.c_str());
                }
                mesh.cross_fill_provider = new SierpinskiFillProvider(mesh.bounding_box, mesh.getSettingInMicrons("infill_line_distance"), mesh.getSettingInMicrons("infill_line_width"));
            }
        }

        // combine infill
        unsigned int combined_infill_layers = std::max(uint64_t(1), round_divide(mesh.getSettingInMicrons("infill_sparse_thickness"), std::max(getSettingInMicrons("layer_height"), (coord_t)1))); //How many infill layers to combine to obtain the requested sparse thickness.
        SkinInfillAreaComputation::combineInfillLayers(mesh, combined_infill_layers);
    }

    // fuzzy skin
    if (mesh.getSettingBoolean("magic_fuzzy_skin_enabled")){
        processFuzzyWalls(mesh);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * processWalls only reads and writes data for the current layer
 */
void FffPolygonGenerator::processWalls(const SliceDataStorage& storage, SliceMeshStorage& mesh, unsigned int layer_nr)
{
    SliceLayer* layer = &mesh.layers[layer_nr];
    WallsComputation walls_computation(storage, mesh, layer_nr);
    walls_computation.generateWalls(storage, mesh, layer);
}

bool FffPolygonGenerator::isEmptyLayer(SliceDataStorage& storage, const unsigned int layer_idx)
{
    if (storage.support.generated && layer_idx < storage.support.supportLayers.size())
    {
        SupportLayer& support_layer = storage.support.supportLayers[layer_idx];
        if (!support_layer.support_infill_parts.empty() || !support_layer.support_bottom.empty() || !support_layer.support_roof.empty())
        {
            return false;
        }
    }
    for (SliceMeshStorage& mesh : storage.meshes)
    {
        SliceLayer& layer = mesh.layers[layer_idx];
        if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") != ESurfaceMode::NORMAL && layer.openPolyLines.size() > 0)
        {
            return false;
        }
        for (const SliceLayerPart& part : layer.parts)
        {
            if (part.print_outline.size() > 0)
            {
                return false;
            }
        }
    }
    return true;
}

void FffPolygonGenerator::removeEmptyFirstLayers(SliceDataStorage& storage, const int layer_height, size_t& total_layers)
{
    int n_empty_first_layers = 0;
    for (size_t layer_idx = 0; layer_idx < total_layers; layer_idx++)
    {
        if (isEmptyLayer(storage, layer_idx))
        {
            n_empty_first_layers++;
        } else
        {
            break;
        }
    }

    if (n_empty_first_layers > 0)
    {
        log("Removing %d layers because they are empty\n", n_empty_first_layers);
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            std::vector<SliceLayer>& layers = mesh.layers;
            layers.erase(layers.begin(), layers.begin() + n_empty_first_layers);
            for (SliceLayer& layer : layers)
            {
                layer.printZ -= n_empty_first_layers * layer_height;
            }
            mesh.layer_nr_max_filled_layer -= n_empty_first_layers;
        }
        total_layers -= n_empty_first_layers;
        storage.support.layer_nr_max_filled_layer -= n_empty_first_layers;
        std::vector<SupportLayer>& support_layers = storage.support.supportLayers;
        support_layers.erase(support_layers.begin(), support_layers.begin() + n_empty_first_layers);
    }
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateSkins read (depend on) data from mesh.layers[*].parts[*].insets and write mesh.layers[n].parts[*].skin_parts
 * generateInfill read mesh.layers[n].parts[*].{insets,skin_parts,boundingBox} and write mesh.layers[n].parts[*].infill_area
 *
 * processSkinsAndInfill read (depend on) mesh.layers[*].parts[*].{insets,boundingBox}.
 *                       write mesh.layers[n].parts[*].{skin_parts,infill_area}.
 */
void FffPolygonGenerator::processSkinsAndInfill(const SliceDataStorage& storage, SliceMeshStorage& mesh, unsigned int layer_nr, bool process_infill)
{
    if (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)
        return;

    SkinInfillAreaComputation skin_infill_area_computation(layer_nr, storage, mesh, process_infill);
    skin_infill_area_computation.generateSkinsAndInfill();

    if (mesh.getSettingBoolean("ironing_enabled") && (!mesh.getSettingBoolean("ironing_only_highest_layer") || (unsigned int)mesh.layer_nr_max_filled_layer == layer_nr))
    {
        // Generate the top surface to iron over.
        mesh.layers[layer_nr].top_surface.setAreasFromMeshAndLayerNumber(mesh, layer_nr);
    }
}

void FffPolygonGenerator::computePrintHeightStatistics(SliceDataStorage& storage)
{
    unsigned int extruder_count = storage.meshgroup->getExtruderCount();

    std::vector<int>& max_print_height_per_extruder = storage.max_print_height_per_extruder;
    assert(max_print_height_per_extruder.size() == 0 && "storage.max_print_height_per_extruder shouldn't have been initialized yet!");
    const int raft_layers = Raft::getTotalExtraLayers(storage);
    max_print_height_per_extruder.resize(extruder_count, -1); //Initialize all as -1.
    { // compute max_object_height_per_extruder
        //Height of the meshes themselves.
        for (SliceMeshStorage& mesh : storage.meshes)
        {
            if (mesh.getSettingBoolean("anti_overhang_mesh") || mesh.getSettingBoolean("support_mesh"))
            {
                continue; //Special type of mesh that doesn't get printed.
            }
            for (unsigned int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
            {
                for (int layer_nr = mesh.layers.size() - 1; layer_nr > max_print_height_per_extruder[extruder_nr]; layer_nr--)
                {
                    if (mesh.getExtruderIsUsed(extruder_nr, layer_nr))
                    {
                        assert(max_print_height_per_extruder[extruder_nr] <= layer_nr);
                        max_print_height_per_extruder[extruder_nr] = layer_nr;
                    }
                }
            }
        }

        //Height of where the support reaches.
        const unsigned int support_infill_extruder_nr = storage.getSettingAsIndex("support_infill_extruder_nr"); // TODO: Support extruder should be configurable per object.
        max_print_height_per_extruder[support_infill_extruder_nr] =
            std::max(max_print_height_per_extruder[support_infill_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);
        const unsigned int support_roof_extruder_nr = storage.getSettingAsIndex("support_roof_extruder_nr"); // TODO: Support roof extruder should be configurable per object.
        max_print_height_per_extruder[support_roof_extruder_nr] =
            std::max(max_print_height_per_extruder[support_roof_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);
        const unsigned int support_bottom_extruder_nr = storage.getSettingAsIndex("support_bottom_extruder_nr"); //TODO: Support bottom extruder should be configurable per object.
        max_print_height_per_extruder[support_bottom_extruder_nr] =
            std::max(max_print_height_per_extruder[support_bottom_extruder_nr],
                     storage.support.layer_nr_max_filled_layer);

        const unsigned int support_roof_skin_extruder_nr = storage.getSettingAsIndex("support_roof_skin_extruder_nr");
        max_print_height_per_extruder[support_roof_skin_extruder_nr] =
            std::max(max_print_height_per_extruder[support_roof_skin_extruder_nr],
                storage.support.layer_nr_max_filled_layer);

        const unsigned int support_bottom_skin_extruder_nr = storage.getSettingAsIndex("support_bottom_skin_extruder_nr");

        max_print_height_per_extruder[support_bottom_skin_extruder_nr] =
            std::max(max_print_height_per_extruder[support_bottom_skin_extruder_nr],
                storage.support.layer_nr_max_filled_layer);

        //Height of where the platform adhesion reaches.
        const EPlatformAdhesion adhesion_type = storage.getSettingAsPlatformAdhesion("adhesion_type");
        switch (adhesion_type)
        {
        case EPlatformAdhesion::SKIRT:
        case EPlatformAdhesion::BRIM:
        {
            const unsigned int skirt_brim_extruder_nr = storage.getSettingAsIndex("skirt_brim_extruder_nr");
            max_print_height_per_extruder[skirt_brim_extruder_nr] = std::max(0, max_print_height_per_extruder[skirt_brim_extruder_nr]); //Includes layer 0.
            break;
        }
        case EPlatformAdhesion::RAFT:
        {
            const unsigned int base_extruder_nr = storage.getSettingAsIndex("raft_base_extruder_nr");
            max_print_height_per_extruder[base_extruder_nr] = std::max(-raft_layers, max_print_height_per_extruder[base_extruder_nr]); //Includes the lowest raft layer.
            const size_t interface_extruder_nr = storage.getSettingAsIndex("raft_interface_extruder_nr");
            max_print_height_per_extruder[interface_extruder_nr] = std::max(-raft_layers + 1, max_print_height_per_extruder[interface_extruder_nr]); //Includes the second-lowest raft layer.
            const size_t surface_extruder_nr = storage.getSettingAsIndex("raft_surface_extruder_nr");
            max_print_height_per_extruder[surface_extruder_nr] = std::max(-1, max_print_height_per_extruder[surface_extruder_nr]); //Includes up to the first layer below the model (so -1).
            break;
        }
        default:
            break; //No adhesion, so no maximum necessary.
        }
    }

    storage.max_print_height_order = order(max_print_height_per_extruder);
    if (extruder_count >= 2)
    {
        int second_highest_extruder = storage.max_print_height_order[extruder_count - 2];
        storage.max_print_height_second_to_last_extruder = max_print_height_per_extruder[second_highest_extruder];
    }
    else
    {
        storage.max_print_height_second_to_last_extruder = -(raft_layers + 1);
    }
}

void FffPolygonGenerator::processOozeShield(SliceDataStorage& storage)
{
    if (!getSettingBoolean("ooze_shield_enabled"))
    {
        return;
    }

    const int ooze_shield_dist = getSettingInMicrons("ooze_shield_dist");

    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.oozeShield.push_back(storage.getLayerOutlines(layer_nr, true).offset(ooze_shield_dist, ClipperLib::jtRound));
    }

    double angle = getSettingInAngleDegrees("ooze_shield_angle");
    if (angle <= 89)
    {
        int allowed_angle_offset = tan(getSettingInAngleRadians("ooze_shield_angle")) * getSettingInMicrons("layer_height"); // Allow for a 60deg angle in the oozeShield.
        for (int layer_nr = 1; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
        {
            storage.oozeShield[layer_nr] = storage.oozeShield[layer_nr].unionPolygons(storage.oozeShield[layer_nr - 1].offset(-allowed_angle_offset));
        }
        for (int layer_nr = storage.max_print_height_second_to_last_extruder; layer_nr > 0; layer_nr--)
        {
            storage.oozeShield[layer_nr - 1] = storage.oozeShield[layer_nr - 1].unionPolygons(storage.oozeShield[layer_nr].offset(-allowed_angle_offset));
        }
    }

    const float largest_printed_area = 1.0; // TODO: make var a parameter, and perhaps even a setting?
    for (int layer_nr = 0; layer_nr <= storage.max_print_height_second_to_last_extruder; layer_nr++)
    {
        storage.oozeShield[layer_nr].removeSmallAreas(largest_printed_area);
    }
}

void FffPolygonGenerator::processDraftShield(SliceDataStorage& storage)
{
    const unsigned int draft_shield_layers = getDraftShieldLayerCount(storage.print_layer_count);
    if (draft_shield_layers <= 0)
    {
        return;
    }
    const int layer_height = getSettingInMicrons("layer_height");

    const unsigned int layer_skip = 500 / layer_height + 1;

    Polygons& draft_shield = storage.draft_protection_shield;
    for (unsigned int layer_nr = 0; layer_nr < storage.print_layer_count && layer_nr < draft_shield_layers; layer_nr += layer_skip)
    {
        draft_shield = draft_shield.unionPolygons(storage.getLayerOutlines(layer_nr, true));
    }

    const int draft_shield_dist = getSettingInMicrons("draft_shield_dist");
    storage.draft_protection_shield = draft_shield.approxConvexHull(draft_shield_dist);
}

void FffPolygonGenerator::processPlatformAdhesion(SliceDataStorage& storage)
{
    const int skirt_brim_extruder_nr = storage.meshgroup->getSettingAsIndex("skirt_brim_extruder_nr");
    const ExtruderTrain* skirt_brim_train = storage.meshgroup->getExtruderTrain(skirt_brim_extruder_nr);
    EPlatformAdhesion adhesion_type = storage.meshgroup->getSettingAsPlatformAdhesion("adhesion_type");
    switch (adhesion_type)
    {
    case EPlatformAdhesion::SKIRT:
    {
        constexpr bool outside_polygons_only = true;
        SkirtBrim::generate(storage, skirt_brim_train->getSettingInMicrons("skirt_gap"), skirt_brim_train->getSettingAsCount("skirt_line_count"), outside_polygons_only);
        break;
    }
    case EPlatformAdhesion::BRIM:
    {
        SkirtBrim::generate(storage, 0, skirt_brim_train->getSettingAsCount("brim_line_count"), skirt_brim_train->getSettingBoolean("brim_outside_only"));
        break;
    }
    case EPlatformAdhesion::RAFT:
    {
        const int raft_extruder_nr = storage.meshgroup->getSettingAsIndex("raft_base_extruder_nr");
        const ExtruderTrain* raft_train = storage.meshgroup->getExtruderTrain(raft_extruder_nr);
        Raft::generate(storage, raft_train->getSettingInMicrons("raft_margin"));
        break;
    }
    case EPlatformAdhesion::NONE:
        break;
    }
}

void FffPolygonGenerator::processFuzzyWalls(SliceMeshStorage& mesh)
{
    if (mesh.getSettingAsCount("wall_line_count") == 0)
    {
        return;
    }
    int64_t fuzziness = mesh.getSettingInMicrons("magic_fuzzy_skin_thickness");
    int64_t avg_dist_between_points = mesh.getSettingInMicrons("magic_fuzzy_skin_point_dist");
    int64_t min_dist_between_points = avg_dist_between_points * 3 / 4; // hardcoded: the point distance may vary between 3/4 and 5/4 the supplied value
    int64_t range_random_point_dist = avg_dist_between_points / 2;
    for (unsigned int layer_nr = 0; layer_nr < mesh.layers.size(); layer_nr++)
    {
        SliceLayer& layer = mesh.layers[layer_nr];
        for (SliceLayerPart& part : layer.parts)
        {
            Polygons results;
            Polygons& skin = (mesh.getSettingAsSurfaceMode("magic_mesh_surface_mode") == ESurfaceMode::SURFACE)? part.outline : part.insets[0];
            for (PolygonRef poly : skin)
            {
                // generate points in between p0 and p1
                PolygonRef result = results.newPoly();

                int64_t dist_left_over = rand() % (min_dist_between_points / 2); // the distance to be traversed on the line before making the first new point
                Point* p0 = &poly.back();
                for (Point& p1 : poly)
                { // 'a' is the (next) new point between p0 and p1
                    Point p0p1 = p1 - *p0;
                    int64_t p0p1_size = vSize(p0p1);
                    int64_t dist_last_point = dist_left_over + p0p1_size * 2; // so that p0p1_size - dist_last_point evaulates to dist_left_over - p0p1_size
                    for (int64_t p0pa_dist = dist_left_over; p0pa_dist < p0p1_size; p0pa_dist += min_dist_between_points + rand() % range_random_point_dist)
                    {
                        int r = rand() % (fuzziness * 2) - fuzziness;
                        Point perp_to_p0p1 = turn90CCW(p0p1);
                        Point fuzz = normal(perp_to_p0p1, r);
                        Point pa = *p0 + normal(p0p1, p0pa_dist) + fuzz;
                        result.add(pa);
                        dist_last_point = p0pa_dist;
                    }
                    dist_left_over = p0p1_size - dist_last_point;

                    p0 = &p1;
                }
                while (result.size() < 3 )
                {
                    unsigned int point_idx = poly.size() - 2;
                    result.add(poly[point_idx]);
                    if (point_idx == 0) { break; }
                    point_idx--;
                }
                if (result.size() < 3)
                {
                    result.clear();
                    for (Point& p : poly)
                        result.add(p);
                }
            }
            skin = results;
        }
    }
}


}//namespace cura
