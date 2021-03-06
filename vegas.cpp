#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include "vegas.h"

namespace vegas {

    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false, false> BGPolygon;  // 注意boost是顺时针的


    double area (Polygon const &input_poly, double scale) {
        BGPolygon po;
        for (auto const &p: input_poly) {
            boost::geometry::append(po.outer(), boost::geometry::model::d2::point_xy<double>(p[0] * scale, p[1] * scale));
        }
        return std::abs(boost::geometry::area(po));
    }

    void contour2polygon_reverse (Polygon const &c, BGPolygon *po) {
        // 逆时针->顺时针
        for (auto const &p: c) {
            boost::geometry::append(po->outer(), boost::geometry::model::d2::point_xy<double>(p[0], p[1]));
        }
    }

    void polygon2contour_reverse (BGPolygon const &po, Polygon *c) {
        // 顺时针->逆时针
        c->clear();
        for (auto const &p: po.outer()) {
            c->push_back(Point{p.x(), p.y()});
        }
    }

    void intersect (Polygon const &o1, Polygon const &o2, Polygon *out) {
        BGPolygon p1; contour2polygon_reverse(o1, &p1);
        BGPolygon p2; contour2polygon_reverse(o2, &p2);
        vector<BGPolygon> outs;
        boost::geometry::intersection(p1, p2, outs);
        BGPolygon best;
        double best_area = 0;   // 找最大的交
        for (auto &p: outs) {
            double a = boost::geometry::area(p);
            if (a > best_area) {
                best_area = a;
                std::swap(best, p);
            }
        }
        out->clear();
        if (best_area > 0) {
            polygon2contour_reverse(best, out);
        }
    }

    Box bound (Polygon const &p) {
        Box b;
        for (auto const &p: b) {
            b.extend(p);
        }
        return b;
    }

    Box bound (Document const &doc) {
        Box b;
        for (auto const &l: doc.layers) {
            for (auto const &s: l.shapes) {
                for (auto const &ll: s.lines) {
                    b.extend(ll[0]);
                    b.extend(ll[1]);
                }
            }
        }
        return b;
    }


    void View::collect (vector<Line> *lines) const {
        vector<Box> roi;
        vector<Box> remove;
        collect_markup_boxes(&roi, Markup::CODE_ROI);
        collect_markup_boxes(&remove, Markup::CODE_REMOVE);
        for (Layer const *l: layers) {
            for (auto const &shape: l->shapes) {
                bool keep = true;
                if (roi.size()) {
                    keep = false;
                    for (auto const &b: roi) {
                        if (b.contains(shape)) {
                            keep = true;
                            break;
                        }
                    }
                }
                for (auto const &b: remove) {
                    if (b.contains(shape)) {
                        keep = false;
                        break;
                    }
                }
                if (!keep) continue;
                lines->insert(lines->end(), shape.lines.begin(), shape.lines.end());
            }
        }
    }
    
    void View::collect_markup_objects (Detection *det) const {
        for (Markup const *m: markups) {
            if (m->code == Markup::CODE_OBJECT_CONTOUR) {
                CHECK(m->label >= 0);
                det->objects.emplace_back();
                auto &obj = det->objects.back();
                obj.label = label;
                obj.contour = m->shape;
                obj.bbox = bound(obj.contour);
            }
        }
    }

    void extract_cc (Document const &doc, vector<Box> *bb, double cc_relax, int pick_layer) {
        // if pick_layers >= 0, only use this layer
		ConnectedComponents cc(cc_relax);
        vector<Box> rects;
        for (int i = 0; i < doc.layers.size(); ++i) {
            if ((pick_layer >= 0) && (i != pick_layer)) continue;
            auto const &layer = doc.layers[i];
            for (auto const &shape: layer.shapes) {
            for (auto const &line: shape.lines) {
				cc.add(line);
			}}
		}
		cc.cleanup();
		cc.swap(bb);
	}

}
