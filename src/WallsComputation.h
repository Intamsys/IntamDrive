//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef WALLS_COMPUTATION_H
#define WALLS_COMPUTATION_H

#include "sliceDataStorage.h"
#include "settings/types/LayerIndex.h"
#include "utils/Coord_t.h"

namespace cura 
{

class SliceLayer;
class SliceLayerPart;
/*!
 * Function container for computing the outer walls / insets / perimeters polygons of a layer
 */
class WallsComputation
{
public:
    /*!
     * \brief Basic constructor initialising the parameters with which to
     * perform the walls computation.
     */
    WallsComputation(const SliceDataStorage& storage, const SliceMeshStorage& mesh, const LayerIndex layer_nr);

    /*!
     * \brief Generates the walls / inner area for all parts in a layer.
     */
    void generateWalls(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayer* layer);

private:

    void generateWalls(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayerPart* part);
    
    void generateRoofingWalls(const SliceDataStorage& storage, const SliceMeshStorage& mesh, SliceLayerPart* part);
    
    /*!
     * Generates the outer inset / perimeter used in spiralize mode for a single layer part. The spiral inset is
     * generated using offsets.
     *
     * \param part The part for which to generate the spiral inset.
     * \param line_width_0 The width of the outer (spiralized) wall.
     * \param wall_0_inset The part for which to generate the spiral inset.
     * \param recompute_outline_based_on_outer_wall Whether we need to recompute the print outline according to the
     *        generated spiral inset.
     */
    void generateSpiralInsets(const SliceDataStorage& storage,  const SliceMeshStorage& mesh, SliceLayerPart* part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall);
private:
    /*!
     * \brief The layer that these walls are generated for.
     */
    const LayerIndex layer_nr;

};
}//namespace cura

#endif//WALLS_COMPUTATION_H
