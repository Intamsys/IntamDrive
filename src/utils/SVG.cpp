/** Copyright (C) 2017 Tim Kuipers - Released under terms of the AGPLv3 License */
#include "SVG.h"


namespace cura {



std::string SVG::toString(Color color)
{
    switch (color)
    {
        case SVG::Color::BLACK: return "black";
        case SVG::Color::WHITE: return "white";
        case SVG::Color::GRAY: return "gray";
        case SVG::Color::RED: return "red";
        case SVG::Color::BLUE: return "blue";
        case SVG::Color::GREEN: return "green";
        case SVG::Color::YELLOW: return "yellow";
        default: return "black";
    }
}



SVG::SVG(const char* filename, AABB aabb, Point canvas_size)
: aabb(aabb)
, aabb_size(aabb.max - aabb.min)
, border(200,100)
, canvas_size(canvas_size)
, scale(std::min(double(canvas_size.X - border.X * 2) / aabb_size.X, double(canvas_size.Y - border.Y * 2) / aabb_size.Y))
{
    output_is_html = strcmp(filename + strlen(filename) - 4, "html") == 0;
    out = fopen(filename, "w");
    if(!out)
    {
        logError("The file %s could not be opened for writing.",filename);
    }
    if (output_is_html)
    {
        fprintf(out, "<!DOCTYPE html><html><body>\n");
    }
    else
    {
        fprintf(out, "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
    }
    fprintf(out, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style=\"width:%llipx;height:%llipx\">\n", canvas_size.X, canvas_size.Y);
}

SVG::~SVG()
{
    fprintf(out, "</svg>\n");
    if (output_is_html)
    {
        fprintf(out, "</body></html>");
    }
    fclose(out);
}

Point SVG::transform(const Point& p) 
{
    return Point((p.X-aabb.min.X)*scale, canvas_size.X - border.X - (p.Y-aabb.min.Y)*scale) + border;
}

void SVG::writeComment(std::string comment)
{
    fprintf(out, "<!-- %s -->\n", comment.c_str());
}

void SVG::writeAreas(const Polygons& polygons, Color color, Color outline_color, coord_t stroke_width) 
{
    for(PolygonsPart& parts : polygons.splitIntoParts())
    {
        for(unsigned int j=0;j<parts.size();j++)
        {
            fprintf(out, "<polygon points=\"");
            for (Point& p : parts[j])
            {
                Point fp = transform(p);
                fprintf(out, "%lli,%lli ", fp.X, fp.Y);
            }
            if (j == 0)
                fprintf(out, "\" style=\"fill:%s;stroke:%s;stroke-width:%lli\" />\n", toString(color).c_str(), toString(outline_color).c_str(), stroke_width);
            else
                fprintf(out, "\" style=\"fill:white;stroke:%s;stroke-width:%lli\" />\n", toString(outline_color).c_str(), stroke_width);
        }
    }
}

void SVG::writeAreas(ConstPolygonRef polygon, Color color, Color outline_color, coord_t stroke_width)
{
    fprintf(out,"<polygon fill=\"%s\" stroke=\"%s\" stroke-width=\"%lli\" points=\"",toString(color).c_str(),toString(outline_color).c_str(), stroke_width); //The beginning of the polygon tag.
    for (const Point& point : polygon) //Add every point to the list of points.
    {
        Point transformed = transform(point);
        fprintf(out,"%lli,%lli ",transformed.X,transformed.Y);
    }
    fprintf(out,"\" />\n"); //The end of the polygon tag.
}

void SVG::writePoint(const Point& p, bool write_coords, int size, Color color)
{
    Point pf = transform(p);
    fprintf(out, "<circle cx=\"%lli\" cy=\"%lli\" r=\"%d\" stroke=\"%s\" stroke-width=\"1\" fill=\"%s\" />\n",pf.X, pf.Y, size, toString(color).c_str(), toString(color).c_str());
    
    if (write_coords)
    {
        fprintf(out, "<text x=\"%lli\" y=\"%lli\" style=\"font-size: 10px;\" fill=\"black\">%lli,%lli</text>\n",pf.X, pf.Y, p.X, p.Y);
    }
}

void SVG::writePoints(ConstPolygonRef poly, bool write_coords, int size, Color color)
{
    for (const Point& p : poly)
    {
        writePoint(p, write_coords, size, color);
    }
}

void SVG::writePoints(Polygons& polygons, bool write_coords, int size, Color color)
{
    for (PolygonRef poly : polygons)
    {
        writePoints(poly, write_coords, size, color);
    }
}

void SVG::writeLines(std::vector<Point> polyline, Color color)
{
    if(polyline.size() <= 1) //Need at least 2 points.
    {
        return;
    }
    
    Point transformed = transform(polyline[0]); //Element 0 must exist due to the check above.
    fprintf(out,"<path fill=\"none\" stroke=\"%s\" stroke-width=\"1\" d=\"M%lli,%lli",toString(color).c_str(),transformed.X,transformed.Y); //Write the start of the path tag and the first endpoint.
    for(size_t point = 1;point < polyline.size();point++)
    {
        transformed = transform(polyline[point]);
        fprintf(out,"L%lli,%lli",transformed.X,transformed.Y); //Write a line segment to the next point.
    }
    fprintf(out,"\" />\n"); //Write the end of the tag.
}

void SVG::writeLine(const Point& a, const Point& b, Color color, int stroke_width)
{
    Point fa = transform(a);
    Point fb = transform(b);
    fprintf(out, "<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:%s;stroke-width:%i\" />\n", fa.X, fa.Y, fb.X, fb.Y, toString(color).c_str(), stroke_width);
}

void SVG::writeLineRGB(const Point& from, const Point& to, int r, int g, int b, int stroke_width)
{
    Point fa = transform(from);
    Point fb = transform(to);
    fprintf(out, "<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" style=\"stroke:rgb(%i,%i,%i);stroke-width:%i\" />\n", fa.X, fa.Y, fb.X, fb.Y, r, g, b, stroke_width);
}

void SVG::writeDashedLine(const Point& a, const Point& b, Color color)
{
    Point fa = transform(a);
    Point fb = transform(b);
    fprintf(out,"<line x1=\"%lli\" y1=\"%lli\" x2=\"%lli\" y2=\"%lli\" stroke=\"%s\" stroke-width=\"1\" stroke-dasharray=\"5,5\" />\n",fa.X,fa.Y,fb.X,fb.Y,toString(color).c_str());
}

void SVG::writeText(Point p, std::string txt, Color color, coord_t font_size)
{
    Point pf = transform(p);
    fprintf(out, "<text x=\"%lli\" y=\"%lli\" style=\"font-size: %llipx;\" fill=\"%s\">%s</text>\n",pf.X, pf.Y, font_size, toString(color).c_str(), txt.c_str());
}
void SVG::writePolygons(const Polygons& polys, Color color, int stroke_width)
{
    for (ConstPolygonRef poly : polys)
    {
        writePolygon(poly, color, stroke_width);
    }
}

void SVG::writePolygon(ConstPolygonRef poly, Color color, int stroke_width)
{
    if (poly.size() == 0)
    {
        return;
    }
    int size = poly.size();
    Point p0 = poly.back();
    int i = 0;
    for (Point p1 : poly)
    {
        if (color == Color::RAINBOW)
        {
            int g = (i * 255 * 11 / size) % (255 * 2);
            if (g > 255) g = 255 * 2 - g;
            int b = (i * 255 * 5 / size) % (255 * 2);
            if (b > 255) b = 255 * 2 - b;
            writeLineRGB(p0, p1, i * 255 / size, g, b, stroke_width);
        }
        else
        {
            writeLine(p0, p1, color, stroke_width);
        }
        p0 = p1;
        i++;
    }
}

} // namespace cura 
