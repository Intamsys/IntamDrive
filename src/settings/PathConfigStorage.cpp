/** Copyright (C) 2017 Ultimaker - Released under terms of the AGPLv3 License */
#include "PathConfigStorage.h"

#include "settings.h" // MAX_INFILL_COMBINE
#include "../sliceDataStorage.h" // SliceDataStorage
#include "../raft.h"

namespace cura
{

std::vector<double> PathConfigStorage::getLineWidthFactorPerExtruder(const SliceDataStorage& storage, int layer_nr)
{
    std::vector<double> ret;
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        if (layer_nr <= 0)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            double factor = train->getSettingAsRatio("initial_layer_line_width_factor");
            ret.push_back(factor);
        }
        else
        {
            ret.push_back(1.0);
        }
    }
    return ret;
}

PathConfigStorage::MeshPathConfigs::MeshPathConfigs(const SliceDataStorage& storage, const SliceMeshStorage& mesh, int layer_thickness, int layer_nr, const std::vector<double>& line_width_factor_per_extruder)
: inset0_config(
    PrintFeatureType::OuterWall
    , mesh.getSettingAsIndex("wall_0_extruder_nr")
    , mesh.getSettingInMicrons("wall_line_width_0") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_0_extruder_nr")]
    , layer_thickness
	, (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("wall_0_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_0"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0")}
    , false // is_bridge_path
    , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("outer_wall_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
    , mesh.getSettingBoolean("override_fan_enable")
)
, insetX_config(
    PrintFeatureType::InnerWall
    , mesh.getSettingAsIndex("wall_x_extruder_nr")
    , mesh.getSettingInMicrons("wall_line_width_x") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_x_extruder_nr")]
    , layer_thickness
	, (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("wall_x_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_wall_x"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x")}
    , false // is_bridge_path
    , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("inner_wall_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
    , mesh.getSettingBoolean("override_fan_enable")
)
, bridge_inset0_config(
    PrintFeatureType::OuterWall
    , mesh.getSettingAsIndex("wall_0_extruder_nr")
    , mesh.getSettingInMicrons("wall_line_width_0") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_0_extruder_nr")]
    , layer_thickness
    , mesh.getSettingAsRatio("bridge_wall_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("bridge_wall_speed"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_0"), mesh.getSettingInMillimetersPerSecond("jerk_wall_0")}
    , true // is_bridge_path
    , mesh.getSettingInPercentage("bridge_fan_speed")
    , false
)
, bridge_insetX_config(
    PrintFeatureType::InnerWall
    , mesh.getSettingAsIndex("wall_x_extruder_nr")
    , mesh.getSettingInMicrons("wall_line_width_x") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("wall_x_extruder_nr")]
    , layer_thickness
    , mesh.getSettingAsRatio("bridge_wall_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("bridge_wall_speed"), mesh.getSettingInMillimetersPerSecond("acceleration_wall_x"), mesh.getSettingInMillimetersPerSecond("jerk_wall_x")}
    , true // is_bridge_path
    , mesh.getSettingInPercentage("bridge_fan_speed")
    , false
)
, top_skin_config(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
	, mesh.getSettingInMicrons("top_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	,(layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("skin_material_flow")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("speed_top"), mesh.getSettingInMillimetersPerSecond("acceleration_top"), mesh.getSettingInMillimetersPerSecond("jerk_top") }
    , false // is_bridge_path
    , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("top_layers_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
    , mesh.getSettingBoolean("override_fan_enable")
)
, bottom_skin_config(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("top_bottom_extruder_nr")
	, mesh.getSettingInMicrons("bottom_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("skin_material_flow")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("speed_bottom"), mesh.getSettingInMillimetersPerSecond("acceleration_bottom"), mesh.getSettingInMillimetersPerSecond("jerk_bottom") }
    , false // is_bridge_path
    , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("bottom_layers_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
    , mesh.getSettingBoolean("override_fan_enable")
)
, top_bridge_skin_config(
    PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
	, mesh.getSettingInMicrons("top_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed"), mesh.getSettingInMillimetersPerSecond("acceleration_top"), mesh.getSettingInMillimetersPerSecond("jerk_top") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed")
    , false
)
, bottom_bridge_skin_config(
    PrintFeatureType::Skin
    , mesh.getSettingAsIndex("top_bottom_extruder_nr")
	, mesh.getSettingInMicrons("bottom_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed"), mesh.getSettingInMillimetersPerSecond("acceleration_bottom"), mesh.getSettingInMillimetersPerSecond("jerk_bottom") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed")
    , false
)
, top_bridge_skin_config2(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
	, mesh.getSettingInMicrons("top_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow_2")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed_2"), mesh.getSettingInMillimetersPerSecond("acceleration_top"), mesh.getSettingInMillimetersPerSecond("jerk_top") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed_2")
    , false
)
, bottom_bridge_skin_config2(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("top_bottom_extruder_nr")
	, mesh.getSettingInMicrons("bottom_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow_2")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed_2"), mesh.getSettingInMillimetersPerSecond("acceleration_bottom"), mesh.getSettingInMillimetersPerSecond("jerk_bottom") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed_2")
    , false
)
, top_bridge_skin_config3(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
	, mesh.getSettingInMicrons("top_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow_3")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed_3"), mesh.getSettingInMillimetersPerSecond("acceleration_top"), mesh.getSettingInMillimetersPerSecond("jerk_top") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed_3")
    , false
)
, bottom_bridge_skin_config3(
	PrintFeatureType::Skin
    , mesh.getSettingAsIndex("top_bottom_extruder_nr")
	, mesh.getSettingInMicrons("bottom_skin_line_width") * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("top_bottom_extruder_nr")]
	, layer_thickness
	, mesh.getSettingAsRatio("bridge_skin_material_flow_3")
	, GCodePathConfig::SpeedDerivatives{ mesh.getSettingInMillimetersPerSecond("bridge_skin_speed_3"), mesh.getSettingInMillimetersPerSecond("acceleration_bottom"), mesh.getSettingInMillimetersPerSecond("jerk_bottom") }
	, true // is_bridge_path
	, mesh.getSettingInPercentage("bridge_fan_speed_3")
    , false
)
, roofing_infill_config(
    PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
    , mesh.getSettingInMicrons("roofing_infill_line_width")* line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("roofing_extruder_nr")]
    , layer_thickness
	, (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("roofing_material_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_roofing_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_roofing"), mesh.getSettingInMillimetersPerSecond("jerk_roofing")}
    , false // is_bridge_path
    , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("top_surface_skin_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
    , mesh.getSettingBoolean("override_fan_enable")
)
, ironing_config(
    PrintFeatureType::Skin
    , mesh.getSettingAsIndex("roofing_extruder_nr")
    , mesh.getSettingInMicrons("top_skin_line_width")
    , layer_thickness
	, (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("ironing_flow")
    , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_ironing"), mesh.getSettingInMillimetersPerSecond("acceleration_ironing"), mesh.getSettingInMillimetersPerSecond("jerk_ironing")}
)
{
    infill_config.reserve(MAX_INFILL_COMBINE);
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        infill_config.emplace_back(
                PrintFeatureType::Infill
                , mesh.getSettingAsIndex("infill_extruder_nr")
                , mesh.getSettingInMicrons("infill_line_width") * (combine_idx + 1) * line_width_factor_per_extruder[mesh.getSettingAsExtruderNr("infill_extruder_nr")]
                , layer_thickness
			    , (layer_nr == 0) ? mesh.getSettingAsRatio("material_flow_layer_0") : mesh.getSettingAsRatio("infill_material_flow")
                , GCodePathConfig::SpeedDerivatives{mesh.getSettingInMillimetersPerSecond("speed_infill"), mesh.getSettingInMillimetersPerSecond("acceleration_infill"), mesh.getSettingInMillimetersPerSecond("jerk_infill")}
                , false // is_bridge_path
                , mesh.getSettingBoolean("override_fan_enable") ? mesh.getSettingInPercentage("infill_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
                , mesh.getSettingBoolean("override_fan_enable")
        );
    }
}

PathConfigStorage::PathConfigStorage(const SliceDataStorage& storage, int layer_nr, int layer_thickness)
: raft_base_extruder_nr(storage.getSettingAsIndex("raft_base_extruder_nr"))
, raft_interface_extruder_nr(storage.getSettingAsIndex("raft_interface_extruder_nr"))
, raft_surface_extruder_nr(storage.getSettingAsIndex("raft_surface_extruder_nr"))
, raft_skin_extruder_nr(storage.getSettingAsIndex("raft_skin_extruder_nr"))
, support_infill_extruder_nr(storage.getSettingAsIndex("support_infill_extruder_nr"))
, support_roof_extruder_nr(storage.getSettingAsIndex("support_roof_extruder_nr"))
, support_bottom_extruder_nr(storage.getSettingAsIndex("support_bottom_extruder_nr"))
, support_roof_skin_extruder_nr(storage.getSettingAsIndex("support_roof_skin_extruder_nr"))
, support_bottom_skin_extruder_nr(storage.getSettingAsIndex("support_bottom_skin_extruder_nr"))
, raft_base_extruder_train(storage.meshgroup->getExtruderTrain(raft_base_extruder_nr))
, raft_interface_extruder_train(storage.meshgroup->getExtruderTrain(raft_interface_extruder_nr))
, raft_surface_extruder_train(storage.meshgroup->getExtruderTrain(raft_surface_extruder_nr))
, raft_skin_extruder_train(storage.meshgroup->getExtruderTrain(raft_skin_extruder_nr))
, support_infill_train(storage.meshgroup->getExtruderTrain(support_infill_extruder_nr))
, support_roof_train(storage.meshgroup->getExtruderTrain(support_roof_extruder_nr))
, support_bottom_train(storage.meshgroup->getExtruderTrain(support_bottom_extruder_nr))
, support_roof_skin_train(storage.meshgroup->getExtruderTrain(support_roof_skin_extruder_nr))
, support_bottom_skin_train(storage.meshgroup->getExtruderTrain(support_bottom_skin_extruder_nr))
, line_width_factor_per_extruder(PathConfigStorage::getLineWidthFactorPerExtruder(storage, layer_nr))
, raft_base_config(
            PrintFeatureType::SupportInterface
            , storage.getSettingAsIndex("raft_base_extruder_nr")
            , raft_base_extruder_train->getSettingInMicrons("raft_base_line_width")
            , raft_base_extruder_train->getSettingInMicrons("raft_base_thickness")
            , (layer_nr == 0)? raft_base_extruder_train->getSettingAsRatio("material_flow_layer_0") : 1.0
            , GCodePathConfig::SpeedDerivatives{raft_base_extruder_train->getSettingInMillimetersPerSecond("raft_base_speed"), raft_base_extruder_train->getSettingInMillimetersPerSecond("raft_base_acceleration"), raft_base_extruder_train->getSettingInMillimetersPerSecond("raft_base_jerk")}
        )
, raft_interface_config(
            PrintFeatureType::Support
            , storage.getSettingAsIndex("raft_interface_extruder_nr")
            , raft_interface_extruder_train->getSettingInMicrons("raft_interface_line_width")
            , raft_interface_extruder_train->getSettingInMicrons("raft_interface_thickness")
            , (layer_nr == 0)? raft_interface_extruder_train->getSettingAsRatio("material_flow_layer_0") : 1.0
            , GCodePathConfig::SpeedDerivatives{raft_interface_extruder_train->getSettingInMillimetersPerSecond("raft_interface_speed"), raft_interface_extruder_train->getSettingInMillimetersPerSecond("raft_interface_acceleration"), raft_interface_extruder_train->getSettingInMillimetersPerSecond("raft_interface_jerk")}
        )
, raft_surface_config(
            PrintFeatureType::SupportInterface
            , storage.getSettingAsIndex("raft_surface_extruder_nr")
            , raft_surface_extruder_train->getSettingInMicrons("raft_surface_line_width")
            , raft_surface_extruder_train->getSettingInMicrons("raft_surface_thickness")
            , (layer_nr == 0)? raft_surface_extruder_train->getSettingAsRatio("material_flow_layer_0") : 1.0
            , GCodePathConfig::SpeedDerivatives{raft_surface_extruder_train->getSettingInMillimetersPerSecond("raft_surface_speed"), raft_surface_extruder_train->getSettingInMillimetersPerSecond("raft_surface_acceleration"), raft_surface_extruder_train->getSettingInMillimetersPerSecond("raft_surface_jerk")}
        )
, raft_skin_config(
    PrintFeatureType::SupportInterface
    , storage.getSettingAsIndex("raft_skin_extruder_nr")
    , raft_skin_extruder_train->getSettingInMicrons("raft_skin_line_width")
    , raft_skin_extruder_train->getSettingInMicrons("raft_skin_thickness")
    , (layer_nr == 0) ? raft_skin_extruder_train->getSettingAsRatio("material_flow_layer_0") : 1.0
    , GCodePathConfig::SpeedDerivatives{ raft_skin_extruder_train->getSettingInMillimetersPerSecond("raft_skin_speed"), raft_surface_extruder_train->getSettingInMillimetersPerSecond("raft_skin_acceleration"), raft_surface_extruder_train->getSettingInMillimetersPerSecond("raft_skin_jerk") }
        )
, support_roof_config(
            PrintFeatureType::SupportInterface
            , storage.getSettingAsIndex("support_roof_extruder_nr")
            , support_roof_train->getSettingInMicrons("support_roof_line_width") * line_width_factor_per_extruder[support_roof_extruder_nr]
            , layer_thickness
            , (layer_nr == 0)? support_roof_train->getSettingAsRatio("material_flow_layer_0") : support_roof_train->getSettingAsRatio("support_roof_material_flow")
            , GCodePathConfig::SpeedDerivatives{support_roof_train->getSettingInMillimetersPerSecond("speed_support_roof"), support_roof_train->getSettingInMillimetersPerSecond("acceleration_support_roof"), support_roof_train->getSettingInMillimetersPerSecond("jerk_support_roof")}
            , false // is_bridge_path
            , support_roof_train->getSettingBoolean("override_fan_enable") ? support_roof_train->getSettingInPercentage("support_roof_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
            , support_roof_train->getSettingBoolean("override_fan_enable")
        )
, support_bottom_config(
            PrintFeatureType::SupportInterface
            , storage.getSettingAsIndex("support_bottom_extruder_nr")
            , support_bottom_train->getSettingInMicrons("support_bottom_line_width") * line_width_factor_per_extruder[support_bottom_extruder_nr]
            , layer_thickness
            , (layer_nr == 0)? support_bottom_train->getSettingAsRatio("material_flow_layer_0") : support_bottom_train->getSettingAsRatio("support_bottom_material_flow")
            , GCodePathConfig::SpeedDerivatives{support_bottom_train->getSettingInMillimetersPerSecond("speed_support_bottom"), support_bottom_train->getSettingInMillimetersPerSecond("acceleration_support_bottom"), support_bottom_train->getSettingInMillimetersPerSecond("jerk_support_bottom")}
            , false // is_bridge_path
            , support_bottom_train->getSettingBoolean("override_fan_enable") ? support_bottom_train->getSettingInPercentage("support_bottom_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
            , support_bottom_train->getSettingBoolean("override_fan_enable")
        )
, support_roof_interface_config(
			PrintFeatureType::SupportRoofInterface
            , storage.getSettingAsIndex("support_roof_skin_extruder_nr")
			, support_roof_train->getSettingInMicrons("support_roof_line_width") * line_width_factor_per_extruder[support_roof_skin_extruder_nr]
			, storage.support.roof_interface_layer_thickness
			, (layer_nr == 0) ? support_roof_skin_train->getSettingAsRatio("material_flow_layer_0") : support_roof_train->getSettingAsRatio("support_roof_material_flow")
			, GCodePathConfig::SpeedDerivatives{ support_roof_skin_train->getSettingInMillimetersPerSecond("speed_support_roof_skin"), support_roof_train->getSettingInMillimetersPerSecond("acceleration_support_roof"), support_roof_train->getSettingInMillimetersPerSecond("jerk_support_roof") }
            , false // is_bridge_path
            , support_roof_skin_train->getSettingBoolean("override_fan_enable") ? support_roof_skin_train->getSettingInPercentage("support_roof_skin_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
            , support_roof_skin_train->getSettingBoolean("override_fan_enable")
        )
, support_bottom_interface_config(
			PrintFeatureType::SupportBottomInterface
            , storage.getSettingAsIndex("support_bottom_skin_extruder_nr")
			, support_bottom_train->getSettingInMicrons("support_bottom_line_width") * line_width_factor_per_extruder[support_bottom_skin_extruder_nr]
			, storage.support.bottom_interface_layer_thickness
			, (layer_nr == 0) ? support_bottom_skin_train->getSettingAsRatio("material_flow_layer_0") : support_bottom_train->getSettingAsRatio("support_bottom_material_flow")
			, GCodePathConfig::SpeedDerivatives{ support_bottom_skin_train->getSettingInMillimetersPerSecond("speed_support_bottom_skin"), support_bottom_train->getSettingInMillimetersPerSecond("acceleration_support_bottom"), support_bottom_train->getSettingInMillimetersPerSecond("jerk_support_bottom") }
            , false // is_bridge_path
            , support_bottom_skin_train->getSettingBoolean("override_fan_enable") ? support_bottom_skin_train->getSettingInPercentage("support_bottom_skin_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
            , support_bottom_skin_train->getSettingBoolean("override_fan_enable")
        )
{
    const int extruder_count = storage.meshgroup->getExtruderCount();
    travel_config_per_extruder.reserve(extruder_count);
    skirt_brim_config_per_extruder.reserve(extruder_count);
    prime_tower_config_per_extruder.reserve(extruder_count);
    for (int extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
        travel_config_per_extruder.emplace_back(
                PrintFeatureType::MoveCombing
                , extruder_nr
                , 0
                , 0
                , 0.0
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("speed_travel"), train->getSettingInMillimetersPerSecond("acceleration_travel"), train->getSettingInMillimetersPerSecond("jerk_travel")}
            );
        skirt_brim_config_per_extruder.emplace_back(
                PrintFeatureType::SkirtBrim
                , extruder_nr
                , train->getSettingInMicrons("skirt_brim_line_width")
                    * ((storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[extruder_nr]) // cause it's also used for the draft/ooze shield
                , layer_thickness
                , (layer_nr == 0)? train->getSettingAsRatio("material_flow_layer_0") : train->getSettingAsRatio("skirt_brim_material_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("skirt_brim_speed"), train->getSettingInMillimetersPerSecond("acceleration_skirt_brim"), train->getSettingInMillimetersPerSecond("jerk_skirt_brim")}
            );
        prime_tower_config_per_extruder.emplace_back(
                PrintFeatureType::SupportInfill
                , extruder_nr
                , train->getSettingInMicrons("prime_tower_line_width")
                    * ((storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[extruder_nr])
                , layer_thickness
                , (layer_nr == 0) ? train->getSettingAsRatio("material_flow_layer_0") : train->getSettingAsRatio("prime_tower_flow")
                , GCodePathConfig::SpeedDerivatives{train->getSettingInMillimetersPerSecond("speed_prime_tower"), train->getSettingInMillimetersPerSecond("acceleration_prime_tower"), train->getSettingInMillimetersPerSecond("jerk_prime_tower")}
            );
    }

    mesh_configs.reserve(storage.meshes.size());
    for (const SliceMeshStorage& mesh_storage : storage.meshes)
    {
        mesh_configs.emplace_back(storage, mesh_storage, layer_thickness, layer_nr, line_width_factor_per_extruder);
    }

    support_infill_config.reserve(MAX_INFILL_COMBINE);
    const float support_infill_line_width_factor = (storage.getSettingAsPlatformAdhesion("adhesion_type") == EPlatformAdhesion::RAFT) ? 1.0 : line_width_factor_per_extruder[support_infill_extruder_nr];
    for (int combine_idx = 0; combine_idx < MAX_INFILL_COMBINE; combine_idx++)
    {
        support_infill_config.emplace_back(
            PrintFeatureType::Support
            , storage.getSettingAsIndex("support_infill_extruder_nr")
            , support_infill_train->getSettingInMicrons("support_line_width") * (combine_idx + 1) * support_infill_line_width_factor
            , layer_thickness
            , (layer_nr == 0)? support_infill_train->getSettingAsRatio("material_flow_layer_0") : support_infill_train->getSettingAsRatio("support_material_flow")
            , GCodePathConfig::SpeedDerivatives{support_infill_train->getSettingInMillimetersPerSecond("speed_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("acceleration_support_infill"), support_infill_train->getSettingInMillimetersPerSecond("jerk_support_infill")}
            , false // is_bridge_path
            , support_infill_train->getSettingBoolean("override_fan_enable") ? support_infill_train->getSettingInPercentage("support_infill_fan_speed") : GCodePathConfig::FAN_SPEED_DEFAULT
            , support_infill_train->getSettingBoolean("override_fan_enable")
        );
    }

    const int initial_speedup_layer_count = storage.getSettingAsCount("speed_slowdown_layers");
    if ((layer_nr >= 0 ||(layer_nr < 0 && layer_nr >= -Raft::getFillerLayerCount(storage))) && layer_nr < initial_speedup_layer_count)
    {
        handleInitialLayerSpeedup(storage, layer_nr, initial_speedup_layer_count);
    }
}

void PathConfigStorage::MeshPathConfigs::smoothAllSpeeds(GCodePathConfig::SpeedDerivatives first_layer_config, int layer_nr, int max_speed_layer)
{
    inset0_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
    insetX_config.smoothSpeed(              first_layer_config, layer_nr, max_speed_layer);
	top_skin_config.smoothSpeed(	        first_layer_config, layer_nr, max_speed_layer);
	bottom_skin_config.smoothSpeed(         first_layer_config, layer_nr, max_speed_layer);
    ironing_config.smoothSpeed(             first_layer_config, layer_nr, max_speed_layer);
    for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
    {
        //Infill speed (per combine part per mesh).
        infill_config[idx].smoothSpeed(first_layer_config, layer_nr, max_speed_layer);
    }
}

void cura::PathConfigStorage::handleInitialLayerSpeedup(const SliceDataStorage& storage, int layer_nr, int initial_speedup_layer_count)
{
    std::vector<GCodePathConfig::SpeedDerivatives> global_first_layer_config_per_extruder;
    global_first_layer_config_per_extruder.reserve(storage.meshgroup->getExtruderCount());
    for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); extruder_nr++)
    {
        const ExtruderTrain* extruder = storage.meshgroup->getExtruderTrain(extruder_nr);
        global_first_layer_config_per_extruder.emplace_back(
            GCodePathConfig::SpeedDerivatives{
                extruder->getSettingInMillimetersPerSecond("speed_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                , extruder->getSettingInMillimetersPerSecond("jerk_print_layer_0")
            });
    }

    { // support
        if (layer_nr < initial_speedup_layer_count)
        {
            const int extruder_nr_support_infill = storage.getSettingAsIndex((layer_nr <= 0)? "support_extruder_nr_layer_0" : "support_infill_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_infill = global_first_layer_config_per_extruder[extruder_nr_support_infill];
            for (unsigned int idx = 0; idx < MAX_INFILL_COMBINE; idx++)
            {
                support_infill_config[idx].smoothSpeed(first_layer_config_infill, std::max(0, layer_nr), initial_speedup_layer_count);
            }

            const int extruder_nr_support_roof = storage.getSettingAsIndex("support_roof_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_roof = global_first_layer_config_per_extruder[extruder_nr_support_roof];
            support_roof_config.smoothSpeed(first_layer_config_roof, std::max(0, layer_nr), initial_speedup_layer_count);
            
			const int extruder_nr_support_bottom = storage.getSettingAsIndex("support_bottom_extruder_nr");
            GCodePathConfig::SpeedDerivatives& first_layer_config_bottom = global_first_layer_config_per_extruder[extruder_nr_support_bottom];
            support_bottom_config.smoothSpeed(first_layer_config_bottom, std::max(0, layer_nr), initial_speedup_layer_count);

			const int extruder_nr_support_roof_interface = storage.getSettingAsIndex("support_roof_skin_extruder_nr");
			GCodePathConfig::SpeedDerivatives& first_layer_config_roof_interface = global_first_layer_config_per_extruder[extruder_nr_support_roof_interface];
			support_roof_interface_config.smoothSpeed(first_layer_config_roof_interface, std::max(0, layer_nr), initial_speedup_layer_count);

			const int extruder_nr_support_bottom_interface = storage.getSettingAsIndex("support_bottom_skin_extruder_nr");
			GCodePathConfig::SpeedDerivatives& first_layer_config_bottom_interface = global_first_layer_config_per_extruder[extruder_nr_support_bottom_interface];
			support_bottom_interface_config.smoothSpeed(first_layer_config_bottom_interface, std::max(0, layer_nr), initial_speedup_layer_count);
        }
    }

    { // extruder configs: travel, skirt/brim (= shield)
        for (int extruder_nr = 0; extruder_nr < storage.meshgroup->getExtruderCount(); ++extruder_nr)
        {
            const ExtruderTrain* train = storage.meshgroup->getExtruderTrain(extruder_nr);
            GCodePathConfig::SpeedDerivatives initial_layer_travel_speed_config{
                    train->getSettingInMillimetersPerSecond("speed_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("acceleration_travel_layer_0")
                    , train->getSettingInMillimetersPerSecond("jerk_travel_layer_0")
            };
            GCodePathConfig& travel = travel_config_per_extruder[extruder_nr];

            travel.smoothSpeed(initial_layer_travel_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);

            // don't smooth speed for the skirt/brim!
            // NOTE: not smoothing skirt/brim means the speeds are also not smoothed for the draft/ooze shield

            const GCodePathConfig::SpeedDerivatives& initial_layer_print_speed_config = global_first_layer_config_per_extruder[extruder_nr];

            GCodePathConfig& prime_tower = prime_tower_config_per_extruder[extruder_nr];
            prime_tower.smoothSpeed(initial_layer_print_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
        }
    }

    { // meshes
        for (unsigned int mesh_idx = 0; mesh_idx < storage.meshes.size(); mesh_idx++)
        {
            const SliceMeshStorage& mesh = storage.meshes[mesh_idx];
            GCodePathConfig::SpeedDerivatives initial_layer_speed_config{
                    mesh.getSettingInMillimetersPerSecond("speed_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("acceleration_print_layer_0")
                    , mesh.getSettingInMillimetersPerSecond("jerk_print_layer_0")
            };

            mesh_configs[mesh_idx].smoothAllSpeeds(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
            mesh_configs[mesh_idx].roofing_infill_config.smoothSpeed(initial_layer_speed_config, std::max(0, layer_nr), initial_speedup_layer_count);
        }
    }
}

}//namespace cura
