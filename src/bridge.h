/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#ifndef BRIDGE_H
#define BRIDGE_H

namespace cura {
    class Polygons;
    class SettingsBaseVirtual;
    class SliceLayer;
    class SliceDataStorage;
    class SupportLayer;

int bridgeAngle(const SettingsBaseVirtual& settings, const Polygons& skin_outline, const SliceDataStorage& storage, const unsigned layer_nr, const unsigned bridge_layer, const SupportLayer* support_layer, Polygons& supported_regions);

}//namespace cura

#endif//BRIDGE_H
