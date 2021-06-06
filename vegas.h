#pragma once

#include <array>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <limits>
#include <fmt/core.h>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>
#include <pybind11/pybind11.h>
#include <glog/logging.h>

namespace vegas {

    namespace py = pybind11;

    using std::array;
    using std::vector;
    using std::map;
    using std::string;
    using std::make_pair;

    // The linear space of 2-D points
    typedef array<double, 2> Point;

    static inline Point round (Point const &p) {
        return Point{std::round(p[0]), std::round(p[1])};
    }

    static inline Point operator + (Point const &p1, Point const &p2) {
        return Point{p1[0] + p2[0], p1[1] + p2[1]};
    }

    static inline Point operator - (Point const &p1, Point const &p2) {
        return Point{p1[0] - p2[0], p1[1] - p2[1]};
    }

    static inline Point operator * (Point const &p1, double v) {
        return Point{p1[0] * v, p1[1] * v};
    }

    static inline Point operator / (Point const &p1, double v) {
        return Point{p1[0] / v, p1[1] / v};
    }

    typedef array<Point, 2> Line;
    typedef vector<Point> Polygon;  // counter-clockwise arranged, 
                                    // same as opencv

    double area (Polygon const &, double scale = 1.0);
    void intersect (Polygon const &, Polygon const &, Polygon *);

    struct Box: public array<Point, 2> {

        Box () {
            (*this)[0][0] = (*this)[0][1] = std::numeric_limits<double>::max();
            (*this)[1][0] = (*this)[1][1] = std::numeric_limits<double>::min();
        }

        Box (Point const &p, double relax) {
            (*this)[0][0] = p[0] - relax;
            (*this)[0][1] = p[1] - relax;
            (*this)[1][0] = p[0] + relax;
            (*this)[1][1] = p[1] + relax;
        }

        Point center () const {
            return ((*this)[0] + (*this)[1]) / 2;
        }

        void contour (Polygon *c) {
            c->clear();
            c->push_back(at(0));
            c->push_back(Point{at(1)[0], at(0)[1]});
            c->push_back(at(1));
            c->push_back(Point{at(0)[0], at(1)[1]});
        }

        double width () const {
            return std::max((*this)[1][0] - (*this)[0][0], 0.0);
        }

        double height () const {
            return std::max((*this)[1][1] - (*this)[0][1], 0.0);
        }

        bool empty () const {
            if ((*this)[0][0] >= (*this)[1][0]) return true;
            if ((*this)[0][1] >= (*this)[1][1]) return true;
            return false;
        }

        void extend (Point p) {
            (*this)[0][0] = std::min((*this)[0][0], p[0]);
            (*this)[0][1] = std::min((*this)[0][1], p[1]);
            (*this)[1][0] = std::max((*this)[1][0], p[0]);
            (*this)[1][1] = std::max((*this)[1][1], p[1]);
        }

        void extend_list (py::list p) {
            extend(Point{p[0].cast<double>(), p[1].cast<double>()});
        }

        void extend_tuple (py::tuple p) {
            extend(Point{p[0].cast<double>(), p[1].cast<double>()});
        }

        void extend_xy (py::object p) {
            extend(Point{p.attr("x").cast<double>(), p.attr("y").cast<double>()});

        }

        py::list unpack () const {
            // convert to list of [lbx, lby, ubx, uby]
            // lb = lower bound
            // ub = upper bound
            py::list v;
            v.append((*this)[0][0]);
            v.append((*this)[0][1]);
            v.append((*this)[1][0]);
            v.append((*this)[1][1]);
            return v;
        }
    };

    static inline Box operator | (Box const &a, Box const &b) {
        Box c;
        c[0][0] = std::min(a[0][0], b[0][0]);
        c[0][1] = std::min(a[0][1], b[0][1]);
        c[1][0] = std::max(a[1][0], b[1][0]);
        c[1][1] = std::max(a[1][1], b[1][1]);
        return c;
    }

    static inline Box operator & (Box const a, Box const &b) {
        Box c;
        c[0][0] = std::max(a[0][0], b[0][0]);
        c[0][1] = std::max(a[0][1], b[0][1]);
        c[1][0] = std::min(a[1][0], b[1][0]);
        c[1][1] = std::min(a[1][1], b[1][1]);
        return c;
    }

    struct Layer {
        string name;
        vector<Line> lines; // all elements are converted
                            // to line segments

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(name);
            ar(lines);
        }
    };

    // A CAD/vector graphics document is a list of layers
    struct Document {
        vector<Layer> layers;

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(layers);
        }

        void load (string const &path) {
            std::ifstream is(path, std::ios::binary);
            cereal::BinaryInputArchive archive(is);
            archive(*this);
        }

        void save (string const &path) {
            std::ofstream os(path, std::ios::binary);
            cereal::BinaryOutputArchive archive(os);
            archive(*this);
        }

    };


    struct Markup {
        int code;
        int layer;          // -1: applicable to all layers
                            // otherwise applicable to specific layer
        Polygon shape;

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(CEREAL_NVP(code), CEREAL_NVP(layer), CEREAL_NVP(shape));
        }
    };

    struct Annotation {
        vector<int> labels;     // layer labels
        vector<Markup> markups;

        template <class Archive>
        void serialize (Archive &ar)
        {
            ar(CEREAL_NVP(labels), CEREAL_NVP(markups));
        }

        void load_json (string const &path) {
            std::ifstream is(path);
            cereal::JSONInputArchive archive(is);
            archive(*this);
        }

        void save_json (string const &path) {
            std::ofstream os(path);
            cereal::JSONOutputArchive archive(os);
            archive(*this);
        }
    };

    class DocumentLoader: public Document {
        // work with vegas.DocumentLoadingCanvas
        vector<Layer> layers;
        map<string, int> lookup;
        int current;
    public:
        DocumentLoader (): current(-1) {
        }

        void select (string const &layer) {
            // select the current layer
            // create if layer does not exist
            auto r = lookup.insert(make_pair(layer, layers.size()));
            if (r.second) {
                layers.emplace_back();
                layers.back().name = layer;
            }
            current = r.first->second;
        }

        void add (double x1, double y1, double x2, double y2) {
            CHECK(current >= 0);
            Line line;
            line[0] = Point{x1, y1};
            line[1] = Point{x2, y2};
            layers[current].lines.push_back(line);
        }
    };

    Box bound (Document const &);
    Box bound (Polygon const &);

    // for object recognition
    struct Object {
        int label = -1;
        Box bbox;
        Polygon contour;
    };


    // 几何运算
    static inline double dot (Point const &p1, Point const &p2) {
        return p1[0] * p2[0] + p1[1] * p2[1];
    }
    
    static inline double norm (Point const &p) {
        return std::sqrt(dot(p, p));
    }

    static inline double distance (Point const &p1, Point const &p2) {
        return norm(p2 - p1);
    }

    static inline Point direction (Point const &p1, Point const &p2) {
        Point p = p2 - p1;
        return p / norm(p);
    }

    static inline double distance (Line const &l, Point const &p) {
        Point dir = direction(l[0], l[1]);
        Point x = p - l[0];
        double a = norm(x);
        double b = dot(dir, x);
        return std::sqrt(a * a - b * b);
    }

    void extract_cc (Document const &doc, vector<Box> *bb, double cc_relax, int pick_layer = -1);

    class Extractor {
    public:
        virtual size_t dim () const = 0;
        virtual void apply (Layer const &layer, double *ft) = 0;
    };
}
