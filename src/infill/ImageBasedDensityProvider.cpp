//Copyright (c) 2017 Tim Kuipers
//Copyright (c) 2018 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#define STBI_FAILURE_USERMSG // enable user friendly bug messages for STB lib
#define STB_IMAGE_IMPLEMENTATION // needed in order to enable the implementation of libs/std_image.h
//#include <stb_image.h>

#include "ImageBasedDensityProvider.h"
#include "SierpinskiFill.h"
#include "../utils/AABB3D.h"
#include "../utils/logoutput.h"

namespace cura {

static constexpr bool diagonal = true;
static constexpr bool straight = false;

ImageBasedDensityProvider::ImageBasedDensityProvider(const std::string filename, const AABB model_aabb)
{
}

ImageBasedDensityProvider::~ImageBasedDensityProvider()
{
}

float ImageBasedDensityProvider::operator()(const AABB3D& query_cube) const
{
		return 0;
}

} // namespace cura
