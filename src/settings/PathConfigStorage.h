/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef SETTINGS_PATH_CONFIGS_H
#define SETTINGS_PATH_CONFIGS_H

#include <vector>

#include "../utils/intpoint.h" // coord_t
#include "../GCodePathConfig.h"

namespace cura
{

class SliceDataStorage; // forward decl for SliceDataStorage
class SliceMeshStorage; // forward decl for SliceDataStorage
class ExtruderTrain; // forward decl for SliceDataStorage

/*!
 * A class to represent all configurations for all features types of printed lines in a meshgroup.
 */
class PathConfigStorage
{
private:
    const unsigned int support_infill_extruder_nr;
    const unsigned int support_roof_extruder_nr;
    const unsigned int support_bottom_extruder_nr;
    const unsigned int support_roof_skin_extruder_nr;
    const unsigned int support_bottom_skin_extruder_nr;
    const ExtruderTrain* support_infill_train;
    const ExtruderTrain* support_roof_train;
    const ExtruderTrain* support_bottom_train;
    const ExtruderTrain* support_roof_skin_train;
    const ExtruderTrain* support_bottom_skin_train;

    const unsigned int raft_base_extruder_nr;
    const unsigned int raft_interface_extruder_nr;
    const unsigned int raft_surface_extruder_nr;
    const unsigned int raft_skin_extruder_nr;
    const ExtruderTrain* raft_base_extruder_train;
    const ExtruderTrain* raft_interface_extruder_train;
    const ExtruderTrain* raft_surface_extruder_train;
    const ExtruderTrain* raft_skin_extruder_train;

    const std::vector<double> line_width_factor_per_extruder;
    static std::vector<double> getLineWidthFactorPerExtruder(const SliceDataStorage& storage, int layer_nr);
public:
    class MeshPathConfigs
    {
    public:
        GCodePathConfig inset0_config;
        GCodePathConfig insetX_config;
        GCodePathConfig bridge_inset0_config;
        GCodePathConfig bridge_insetX_config;

		GCodePathConfig top_skin_config;
		GCodePathConfig bottom_skin_config;

		GCodePathConfig top_bridge_skin_config;
		GCodePathConfig top_bridge_skin_config2;
		GCodePathConfig top_bridge_skin_config3;

		GCodePathConfig bottom_bridge_skin_config;
		GCodePathConfig bottom_bridge_skin_config2;
		GCodePathConfig bottom_bridge_skin_config3;

        GCodePathConfig roofing_infill_config;

        std::vector<GCodePathConfig> infill_config;
        GCodePathConfig ironing_config;
        MeshPathConfigs(const SliceDataStorage& storage, const SliceMeshStorage& mesh, int layer_thickness, int layer_nr, const std::vector<double>& line_width_factor_per_extruder);
        void smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer);
    };

    GCodePathConfig raft_base_config;
    GCodePathConfig raft_interface_config;
    GCodePathConfig raft_surface_config;
    GCodePathConfig raft_skin_config;

    std::vector<GCodePathConfig> travel_config_per_extruder; //!< The config used for travel moves (only speed is set!)
    std::vector<GCodePathConfig> skirt_brim_config_per_extruder; //!< Configuration for skirt and brim per extruder.
    std::vector<GCodePathConfig> prime_tower_config_per_extruder; //!< Configuration for the prime tower per extruder.

    std::vector<GCodePathConfig> support_infill_config; //!< The config used to print the normal support, rather than the support interface
    GCodePathConfig support_roof_config; //!< The config used to print the dense roofs of support.
    GCodePathConfig support_bottom_config; //!< The config to use to print the dense bottoms of support

	GCodePathConfig support_roof_interface_config;
	GCodePathConfig support_bottom_interface_config;

    std::vector<MeshPathConfigs> mesh_configs; //!< For each meash the config for all its feature types

    /*!
     * \warning Note that the layer_nr might be below zero for raft (filler) layers
     */
    PathConfigStorage(const SliceDataStorage& storage, int layer_nr, int layer_thickness);

private:
    void handleInitialLayerSpeedup(const SliceDataStorage& storage, int layer_nr, int initial_speedup_layer_count);
};

}; // namespace cura

#endif // SETTINGS_PATH_CONFIGS_H
