/** Copyright (C) 2013 Ultimaker - Released under terms of the AGPLv3 License */
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"
#include "pathPlanning/LinePolygonsCrossings.h"
#include "pathPlanning/CombPath.h"

#define INLINE static inline

namespace cura {

/**
*
*/

static inline bool pointsAreCoincident(const Point& a, const Point& b)
{
    return vSize2(a - b) < 25; // points are closer than 5uM, consider them coincident
}

/**
*
*/
void LineOrderOptimizer::optimize(bool find_chains)
{
    int gridSize = 5000; // the size of the cells in the hash grid. TODO
    SparsePointGridInclusive<unsigned int> line_bucket_grid(gridSize);
    //bool picked[polygons.size()];
	bool *picked = new bool[polygons.size()];
    memset(picked, false, sizeof(bool) * polygons.size());/// initialized as falses

    loc_to_line = nullptr;

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best_point_idx = -1;
        float best_point_dist = std::numeric_limits<float>::infinity();
        ConstPolygonRef poly = *polygons[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++) /// get closest point from polygon
        {
            float dist = vSize2f(poly[point_idx] - startPoint);
            if (dist < best_point_dist)
            {
                best_point_idx = point_idx;
                best_point_dist = dist;
            }
        }
        polyStart.push_back(best_point_idx);

        assert(poly.size() == 2);

        line_bucket_grid.insert(poly[0], poly_idx);
        line_bucket_grid.insert(poly[1], poly_idx);

    }

    Point incoming_perpundicular_normal(0, 0);
    Point prev_point = startPoint;
    bool have_chains = find_chains; // if find_chains is true, assume that line segments are chained together (i.e. zigzags) and set to false later if no chains exist
    for (unsigned int order_idx = 0; order_idx < polygons.size(); order_idx++) /// actual path order optimizer
    {
        int best_line_idx = -1;
        float best_score = std::numeric_limits<float>::infinity(); // distance score for the best next line

        // for the first line we would prefer a line that is at the end of a sequence of connected lines (think zigzag) and
        // so we only consider the closest line when looking for the second line onwards
        if (order_idx > 0)
        {
            /// check if single-line-polygon is close to last point
            for(unsigned int close_line_idx : line_bucket_grid.getNearbyVals(prev_point, gridSize))
            {
                if (picked[close_line_idx] || polygons[close_line_idx]->size() < 1)
                {
                    continue;
                }
                updateBestLine(close_line_idx, best_line_idx, best_score, prev_point, incoming_perpundicular_normal);
            }
        }

        if (best_line_idx != -1 && best_score > (2 * gridSize * gridSize))
        {
            // we found a point that is close to prev_point as the crow flies but the score is high so it must have been
            // penalised due to the part boundary clashing with the straight line path so let's forget it and find something closer
            best_line_idx = -1;
            best_score = std::numeric_limits<float>::infinity();
        }

        if (best_line_idx != -1 && have_chains && !pointsAreCoincident(prev_point, (*polygons[best_line_idx])[polyStart[best_line_idx]]))
        {
            // we found a point close to prev_point but it's not close enough for the points to be considered coincident so we would
            // probably be better off by ditching this point and finding an end of a chain instead (let's hope it's not too far away!)
            best_line_idx = -1;
            best_score = std::numeric_limits<float>::infinity();
        }

        // if no line ends close to prev_point, see if we can find a point on a line that could be the start of a chain of lines
        if (best_line_idx == -1 && have_chains)
        {
            have_chains = false; // now assume that we don't have any chains and change back to true below if we find any joined line segments
            for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
            {
                if (picked[poly_idx] || polygons[poly_idx]->size() < 1) /// skip single-point-polygons
                {
                    continue;
                }
                assert(polygons[poly_idx]->size() == 2);

                // does this line either end in thin air (doesn't join another line) or join another line that has already been picked?
                // check both of its ends and see if it's a possible candidate to be used to start the next sequence
                for (unsigned point_idx = 0; point_idx < 2; ++point_idx)
                {
                    int num_joined_lines = 0;
                    const Point& p = (*polygons[poly_idx])[point_idx];
                    // look at each of the lines that finish close to this line to see if either of its vertices are coincident this vertex
                    for (unsigned int close_line_idx : line_bucket_grid.getNearbyVals(p, gridSize))
                    {
                        if (close_line_idx != poly_idx && (pointsAreCoincident(p, (*polygons[close_line_idx])[0]) || pointsAreCoincident(p, (*polygons[close_line_idx])[1])))
                        {
                            have_chains = true; // we have found a joint between line segments so we have chains

                            ++num_joined_lines;

                            if (picked[close_line_idx])
                            {
                                // candidate line exactly meets a line that has already been picked so consider this vertex as a start point
                                updateBestLine(poly_idx, best_line_idx, best_score, prev_point, incoming_perpundicular_normal, point_idx);
                            }
                        }
                    }
                    if (num_joined_lines == 0)
                    {
                        // candidate line ends in thin air so this vertex could possibly be located at the end of a sequence of lines
                        updateBestLine(poly_idx, best_line_idx, best_score, prev_point, incoming_perpundicular_normal, point_idx);
                    }
                }
            }
        }

        // fallback to using the nearest unpicked line
        if (best_line_idx == -1) /// if single-line-polygon hasn't been found yet
        {
            for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
            {
                if (picked[poly_idx] || polygons[poly_idx]->size() < 1) /// skip single-point-polygons
                {
                    continue;
                }
                assert(polygons[poly_idx]->size() == 2);

                updateBestLine(poly_idx, best_line_idx, best_score, prev_point, incoming_perpundicular_normal);

            }
        }

        if (best_line_idx > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            ConstPolygonRef best_line = *polygons[best_line_idx];
            assert(best_line.size() == 2);

            int line_start_point_idx = polyStart[best_line_idx];
            int line_end_point_idx = line_start_point_idx * -1 + 1; /// 1 -> 0 , 0 -> 1
            const Point& line_start = best_line[line_start_point_idx];
            const Point& line_end = best_line[line_end_point_idx];
            prev_point = line_end;
            incoming_perpundicular_normal = turn90CCW(normal(line_end - line_start, 1000));

            picked[best_line_idx] = true;
            polyOrder.push_back(best_line_idx);
        }
        else
        {
            logError("Failed to find next closest line.\n");
        }
    }
    if (loc_to_line != nullptr)
    {
        delete loc_to_line;
    }
}

float LineOrderOptimizer::combingDistance2(const Point &p0, const Point &p1)
{
    if (loc_to_line == nullptr)
    {
        // do the initialisation required to be able to calculate realistic travel distances to the start of new paths
        loc_to_line = PolygonUtils::createLocToLineGrid(*combing_boundary, 1000); // 1mm grid to reduce computation time
    }

    CombPath comb_path;
    if (LinePolygonsCrossings::comb(*combing_boundary, *loc_to_line, p0, p1, comb_path, -40, 0, false))
    {
        float dist = 0;
        Point last_point = p0;
        for (const Point& comb_point : comb_path)
        {
            dist += vSize(comb_point - last_point);
            last_point = comb_point;
        }
        return dist * dist;
    }

    // couldn't comb, fall back to a large distance

    return vSize2f(p1 - p0) * 10000;
}

inline void LineOrderOptimizer::updateBestLine(unsigned int poly_idx, int& best, float& best_score, Point prev_point, Point incoming_perpundicular_normal, int just_point)
{
    // when looking for chains, just_point will be either 0 or 1 depending on which vertex we are currently interested in testing
    // if just_point is -1, it means that we are not looking for chains and we will test both vertices to see if either is best

    const Point& p0 = (*polygons[poly_idx])[0];
    const Point& p1 = (*polygons[poly_idx])[1];
    float dot_score = (just_point >= 0) ? 0 : getAngleScore(incoming_perpundicular_normal, p0, p1);

    if (just_point != 1)
    { /// check distance to first point on line (0)
        float score = vSize2f(p0 - prev_point) + dot_score; // prefer 90 degree corners
        if (score < best_score
            && combing_boundary != nullptr
            && !pointsAreCoincident(p0, prev_point)
            && PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p0, prev_point))
        {
            score = combingDistance2(p0, prev_point);
        }
        if (score < best_score)
        {
            best = poly_idx;
            best_score = score;
            polyStart[poly_idx] = 0;
        }
    }
    if (just_point != 0)
    { /// check distance to second point on line (1)
        float score = vSize2f(p1 - prev_point) + dot_score; // prefer 90 degree corners
        if (score < best_score
            && combing_boundary != nullptr
            && !pointsAreCoincident(p1, prev_point)
            && PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p1, prev_point))
        {
            score = combingDistance2(p1, prev_point);
        }
        if (score < best_score)
        {
            best = poly_idx;
            best_score = score;
            polyStart[poly_idx] = 1;
        }
    }
}

float LineOrderOptimizer::getAngleScore(Point incoming_perpundicular_normal, Point p0, Point p1)
{
    return dot(incoming_perpundicular_normal, normal(p1 - p0, 1000)) * 0.0001f;
}


}//namespace cura
